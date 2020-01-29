//
// Created by hejia on 7/30/19.
//
#include <fstream>

#include <boost/thread/mutex.hpp>

#include "wecook/TaskGraph/TaskGraph.h"
#include "wecook/World.h"
#include "wecook/utils.h"
#include "wecook/Agents/Robot.h"
#include "wecook/PrimitiveTaskGraph/PrimitiveTaskGraph.h"

using namespace wecook;

int World::addTask(wecook::Task &task) {
  m_tasks.emplace_back(task);
}

void World::initialize() {
  m_env = std::make_shared<aikido::planner::World>("wecook");
  m_viewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(m_env, "wecook", "map");
  m_viewer->setAutoUpdate(true);
  ros::Duration(1.).sleep();
}

void World::stop() {
  m_isEnd = true;
  m_thread.join();
}

void World::recording() {
  // record time too
  ros::Time begin = ros::Time::now();
  std::vector<std::vector<double>> vecJoints;
  double waitingTime = 0.3;
  while (!m_recordingEnd) {
    // while recording is not end, we keep recording
    for (const auto &agent : m_agents) {
      // don't need so many states
      ros::Duration(waitingTime).sleep();
      auto adaImg = std::dynamic_pointer_cast<Robot, Agent>(agent.second)->m_adaImg;
      // first get arm positions
//      auto wholeJoints = adaImg->getMetaSkeleton()->getPositions();
      auto armJoints = adaImg->getArm()->getMetaSkeleton()->getPositions();
//      auto handJoints = ada->getHand()->getMetaSkeleton()->getPositions();
//      Eigen::VectorXd joints(armJoints.size() + handJoints.size());
//      joints << armJoints, handJoints;
      std::vector<double> newJoints;
      newJoints.resize(armJoints.size());
      Eigen::VectorXd::Map(&newJoints[0], armJoints.size()) = armJoints;
      if (not vecJoints.empty() && vecJoints.back() == newJoints) {
        ROS_INFO("Switch to longer waiting time...");
        waitingTime = 1.5;
      } else {
        waitingTime = 0.3;
      }
      vecJoints.emplace_back(newJoints);
    }
  }
  ros::Time end = ros::Time::now();
  ROS_INFO_STREAM("Started recording at: " << begin << "; Finished recording at: " << end);
  ROS_INFO_STREAM("We have: " << vecJoints.size() << " points");

  ROS_INFO("Caching");
  std::ofstream cachingFile;
  cachingFile.open("cached.txt");
  for (const auto &joints : vecJoints) {
    for (const auto &position : joints) {
      cachingFile << position << ' ';
    }
    cachingFile << '\n';
  }
  cachingFile.close();
}

void World::run() {
  while (!m_isEnd) {
    if (!m_tasks.empty()) {
      m_isFree = false;
      Task task = m_tasks[0]; // get the first element in task queue
      // We have two modes planning and following
      auto taskType = task.getTaskType();
      if (taskType == Task::Planning) {
        // TODO
      } else {
        // Do following mode
        ROS_INFO("Setting up the environment...");
        setupFollowingTask(task);
        
        // PK_TODO: Hack: apriltags notification is coming a little late and robots are asking
        // for object position before tag notifications are coming. Sleeping a bit to adjust for that.
        // Should figure out a better plan?
        ros::Duration(1).sleep();

        // After setting up the environment, we want to start a skeleton state recording thread
        m_recordingEnd = false;
        boost::thread *recordingThread;
        // PK_TODO: There was an issue with running recording in real robot demo, this was done till the issue
        // is fully figured out.
        bool runRecording = false; //task.ifSim()
        if (runRecording) {
            recordingThread = new boost::thread(&World::recording, this);
        }

        std::vector<Action> subgoals = task.getSubgoals();
        ROS_INFO("Building the task graph...");
        auto taskGraph = std::make_shared<TaskGraph>(subgoals);

        // transfer taskGraph to primitive taskGraph
        ROS_INFO("Compiling the task graph...");
        m_actionPlanner.compile(taskGraph, m_agents, m_containingMap, m_objectMgr);

        boost::mutex motionMutex;

        // now dispatch taskgraph to all agents
        ROS_INFO("Dispatching the task to agents...");
        for (const auto &agent : m_agents) {
          auto taskExecutorThread = std::make_shared<TaskExecutorThread>(agent.second,
                                                                         m_primitiveActionExecutor,
                                                                         taskGraph);
          if (task.getMotionPlannerType() == Task::RRTConnect) {
            taskExecutorThread->setMotionMutex(&motionMutex);
          }
          m_mapTaskExecutorThread.insert(std::make_pair(agent.first, taskExecutorThread));
        }

        // wait for agents to be free
        for (const auto &taskExecutorThread : m_mapTaskExecutorThread) {
          while (!taskExecutorThread.second->isEnd()) {
            ros::Duration(0.5).sleep();
          }
        }
        // After executing we want to stop recording and cache the trajectory
        m_recordingEnd = true;
        if (runRecording) {
            recordingThread->join();
        }

        cleanFollowingTask(task);
        m_tasks.pop_back();
      }
    }
    m_isFree = true;
  }
}

void World::setupFollowingTask(const Task &task) {
  // First add agents to the world
  auto agents = task.getAgents();

  for (const auto &agent : agents) {
    // create each agent
    std::vector<double> pose = agent.m_pose;
    auto transform = vectorToIsometry(pose);
    if (agent.m_type == "r") {
      // This is a robot agent
      auto pAgent = std::make_shared<Robot>(transform, agent.m_pid, task.ifSim(), agent.m_if_float);
      m_agents.emplace(std::pair<std::string, std::shared_ptr<Agent>>(agent.m_pid, pAgent));
    }
  }

  for (const auto &agent : m_agents) {
    agent.second->init(m_env);
  }

  std::vector<Object> objects = task.getObjects();
  auto tags = task.getTags();

  m_objectMgr = std::make_shared<ObjectMgr>(m_nh);

  m_objectMgr->init(objects, tags, task.ifSim(), m_env);

  m_containingMap = std::make_shared<ContainingMap>(task, m_objectMgr, m_env);

  // intialize primitive action executor
  m_primitiveActionExecutor = std::make_shared<PrimitiveActionExecutor>(m_agents, m_objectMgr, m_containingMap);
}

void World::cleanFollowingTask(const Task &task) {
  std::vector<Object> objects = task.getObjects();
  auto tags = task.getTags();

  m_primitiveActionExecutor.reset();

  m_containingMap->unconnectAll();
  m_containingMap.reset();

  m_objectMgr->clear(objects, tags, m_env);
  m_objectMgr.reset();

  m_mapTaskExecutorThread.clear();

  // Remove all agents
  for (auto &agent : m_agents) {
    // TODO stop trajectory controller
    agent.second->end();
  }
}

void World::setupPlanningTask(const Task &task) {
  // 1. Parse PDDL files
  auto pddlDomain = task.getPDDLDomain();
  auto pddlProblem = task.getPDDLProblem();

  // TODO
}

void World::cleanPlanningTask(const Task &task) {
  // TODO
}