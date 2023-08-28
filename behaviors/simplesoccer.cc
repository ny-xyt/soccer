#include "simplesoccer.h"
#include <cmath>  // 用于数学函数
#include <unordered_map>
#include <vector>

SimpleSoccerBehavior::SimpleSoccerBehavior(const std::string teamName,
                                           int uNum,
                                           const map<string, string> &namedParams_,
                                           const string &rsg_)
    : NaoBehavior(teamName,
                  uNum,
                  namedParams_,
                  rsg_)
{
}
   int h=0;
   who_close who_close_t;
  WorldModel* worldModel;
void SimpleSoccerBehavior::beam(double &beamX, double &beamY, double &beamAngle)
{ 
    VecPosition space = getSpaceForRole(static_cast<Role>(worldModel->getUNum() - 1));
    if (worldModel->getUNum() == 5 || worldModel->getUNum() == 6)
    {
        beamX = -HALF_FIELD_X / 2+2;
        if (worldModel->getUNum() == 5)
        {
            beamY = 0 + HALF_FIELD_Y / 2;
        }
        else
        {
            beamY = 0 - HALF_FIELD_Y / 2;
        }
        beamAngle = 0;
    }
    else
    {
        beamX = space.getX();
        beamY = space.getY();
       beamAngle = 0;
    }
}


SkillType SimpleSoccerBehavior::selectSkill()
{
   // 创建一个 WorldModel 对象，这里假设您有一个 WorldModel 类
    WorldModel worldModel;
// 创建一个中央控制器对象
CentralController controller = CentralController();
    // 在适当的时候调用各个函数
     // 在这里调用更新敌方球员位置信息的函数
 worldModel.updateOpponentPositions();
    // 先调用 assignStrategyAndPriorityByRoleAndState
    controller.assignStrategyAndPriorityByRoleAndState(&worldModel);
    // 然后调用 adjustMatchMatrix
    controller.adjustMatchMatrix(&worldModel);
// 在每个周期里调用act函数
controller.act(worldModel);
    // 定义并初始化 newMode 变量
  StrategyMode newMode = MODE_ATTACK;
  // 根据需要切换策略模式
controller.switchStrategyMode(newMode);
// std::map<int, Role> agentRoles = getAgentRoles();

//    /* Draw role assignments该部分代码实现绘制球员要去的地方
//     worldModel->getRVSender()->clear(); // erases drawings from previous cycle
//     VecPosition space = getSpaceForRole(agentRoles[worldModel->getUNum()]);
//     worldModel->getRVSender()->drawLine("me_to_space", me.getX(), me.getY(), space.getX(), space.getY());
//     worldModel->getRVSender()->drawPoint("role_space", space.getX(), space.getY(), 10.0f);
//     */

//    if (agentRoles[worldModel->getUNum()] == ON_BALL)//判断自己角色是不是控球角色
//     {
//         if (me.getDistanceTo(ball) > 1) //判断自己和球距离是不是大于1，me.getDistanceTo(ball) 判断球和自己距离
//         {
//             // Walk to ball
//             return goToTarget(ball);  //如果不够靠近，就靠近球
//         }
//         else
//         {
//             // Kick ball toward opponent's goal
//             return kickBall(KICK_FORWARD, VecPosition(HALF_FIELD_X, 0, 0)); //向对方球门踢球
//         }
//     }
//     return goToSpace(getSpaceForRole(agentRoles[worldModel->getUNum()]));
}


int SimpleSoccerBehavior::getPlayerClosestToBall()   //该函数实现找到最靠近球的球员并返回他的编号
{
    // Find closest player to ball
    int playerClosestToBall = -1;//球员编号
    double closestDistanceToBall = 10000; //定义距离
    for (int i = WO_TEAMMATE1; i < WO_TEAMMATE1 + NUM_AGENTS; ++i) //遍历所有队员编号
    {
        VecPosition temp;// VecPosition类型的temp,存放当前遍历到的球员的位置
        int playerNum = i - WO_TEAMMATE1 + 1; //
        if (worldModel->getUNum() == playerNum) //判断当前球员是不是自己
        {
            // temp记录位置球员位置
            temp = worldModel->getMyPosition();
        }
        else
        {  
            WorldObject *teammate = worldModel->getWorldObject(i);//获取当前球员的worldobject对象
            if (teammate->validPosition)  //判断位置是否有效
            {
                temp = teammate->pos;  //有效将位置赋予给temp
            }
            else
            {
                continue;  //无效寻找下一个球员
            }
        }
        temp.setZ(0); //只考虑平面

        double distanceToBall = temp.getDistanceTo(ball); //计算球员到球的距离
        if (distanceToBall < closestDistanceToBall) //遍历所有其球员后找到最靠近的球员
        {
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }
    return playerClosestToBall;
}

map<int, Role> SimpleSoccerBehavior::getAgentRoles()//给每个机器人分配一个角色
{
    map<int, Role> agentRoles;   //map<int, Role> 一种类型
    set<Role> assignedRoles;        
    agentRoles[getPlayerClosestToBall()] = ON_BALL; //寻找离球最近的一个球员，为他分配on_ball角色
    assignedRoles.insert(ON_BALL);
    if (!agentRoles.count(1))
    {
        // Assign goalie the goalie role if goalie is not already assigned a role
        agentRoles[1] = GOALIE;
        assignedRoles.insert(GOALIE);
    }
    // Simple greedy role assignment of remaining unassigned roles
    typedef std::list<std::pair<double, std::pair<int, Role>>> dist_to_roles_list;
    dist_to_roles_list agentsDistancesToRoles;
    for (int r = 0; r < NUM_ROLES; r++)
    {
        Role role = static_cast<Role>(r);
        if (assignedRoles.count(role))
        {
            continue;
        }
        for (int i = 1; i <= NUM_AGENTS; i++)
        {
            if (agentRoles.count(i))
            {
                continue;
            }
            agentsDistancesToRoles.push_back(make_pair(getAgentDistanceToRole(i, role), make_pair(i, role)));
        }
    }

    agentsDistancesToRoles.sort();

    for (dist_to_roles_list::iterator it = agentsDistancesToRoles.begin(); it != agentsDistancesToRoles.end(); ++it)
    {
        pair<int, Role> assignment = it->second;
        int agent = assignment.first;
        Role role = assignment.second;
        if (agentRoles.count(agent) || assignedRoles.count(role))
        {
            continue;
        }
        agentRoles[agent] = role;
        assignedRoles.insert(role);
        if (agentRoles.size() == NUM_AGENTS)
        {
            break;
        }
    }

    return agentRoles;
}

VecPosition SimpleSoccerBehavior::getSpaceForRole(Role role)//分配角色后让球员根据不同角色位置调整位置
{
    VecPosition ball = worldModel->getBall();
    if (beamablePlayMode())
    {
        ball = VecPosition(0, 0, 0);
    }
    ball.setZ(0);

    // Keep ball position on the field
    ball.setX(max(min(ball.getX(), HALF_FIELD_X), -HALF_FIELD_X));
    ball.setY(max(min(ball.getY(), HALF_FIELD_Y), -HALF_FIELD_Y));

    VecPosition space = VecPosition(0, 0, 0);

    switch (role)
    {
    case ON_BALL:
        space = ball;
        break;
    case GOALIE:
        space = VecPosition(-HALF_FIELD_X + 0.5, 0, 0);
        break;
    case SUPPORTER:
        space = ball + VecPosition(-2, 0, 0);
        break;
    case BACK_LEFT:
        space = ball + VecPosition(-10, 3, 0);
        break;
    case BACK_RIGHT:
        space = ball + VecPosition(-10, -3, 0);
        break;
    case MID_LEFT:
        space = ball + VecPosition(-1, 2, 0);
        break;
    case MID_RIGHT:
        space = ball + VecPosition(-1, -2, 0);
        break;
    case WING_LEFT:
        space = ball + VecPosition(0, 7, 0);
        break;
    case WING_RIGHT:
        space = ball + VecPosition(0, -7, 0);
        break;
    case FORWARD_LEFT:
        space = ball + VecPosition(5, 3, 0);
        break;
    case FORWARD_RIGHT:
        space = ball + VecPosition(5, -3, 0);
        break;
    default:
        cerr << "Unknown role: " << role << "\n";
    }

    space.setX(max(-HALF_FIELD_X, min(HALF_FIELD_X, space.getX())));
    space.setY(max(-HALF_FIELD_Y, min(HALF_FIELD_Y, space.getY())));

    if (beamablePlayMode())
    {
        // Stay within your own half on kickoffs
        space.setX(min(-0.5, space.getX()));
    }

    return space;
}

double SimpleSoccerBehavior::getAgentDistanceToRole(int uNum, Role role)//计算指定编号机器人和职位的位置
{
    VecPosition temp;
    if (worldModel->getUNum() == uNum)
    {
        // This is us
        temp = worldModel->getMyPosition();
    }
    else
    {
        WorldObject *teammate = worldModel->getWorldObject(WO_TEAMMATE1 + uNum - 1);
        temp = teammate->pos;
    }
    temp.setZ(0);

    double distanceToRole = temp.getDistanceTo(getSpaceForRole(role));
    return distanceToRole;
}

SkillType SimpleSoccerBehavior::goToSpace(VecPosition space)//机器人移动到某个位置
{
    const double SPACE_THRESH = 0.5;
    if (me.getDistanceTo(space) < SPACE_THRESH)
    {
        return watchPosition(ball);
    }

    // Adjust target to not be too close to teammates or the ball
    VecPosition target = collisionAvoidance(true /*teammate*/, false /*opponent*/, true /*ball*/, 1 /*proximity thresh*/, .5 /*collision thresh*/, space, true /*keepDistance*/);
    return goToTarget(target);
}

//SkillType SimpleSoccerBehavior::watchPosition(VecPosition pos)
// {
//     const double POSITION_CENTER_THRESH = 10;
//     VecPosition localPos = worldModel->g2l(pos);
//     SIM::AngDeg localPosAngle = atan2Deg(localPos.getY(), localPos.getX());
//     if (abs(localPosAngle) > POSITION_CENTER_THRESH)
//     {
//         return goToTargetRelative(VecPosition(), localPosAngle);
//     }
//     return SKILL_STAND;
// }
//根据球员的编号来获取球员的位置
 VecPosition getPlayerPosition(int playerNum) //获取编号球员的位置
  {
        // 使用您的代码中的相应方法来获取球员位置，这里只是一个示例
        return worldModel->GetMyPosition(playerNum);
    }

 double calculateDistanceToEnemyGoal(int dribblerNum)//该函数用于计算制定编号的球员位置与敌方球门位置之间的距离
     {
        VecPosition dribblerPos = getPlayerPosition(dribblerNum);
        VecPosition enemyGoalPos(HALF_FIELD_X, 0, 0);  // 敌方球门位置，根据实际情况修改
        return dribblerPos.getDistanceTo(enemyGoalPos);
    }

double calculateDistanceToBall(int playerNum)    // 计算制定编号球员与球之间的距离
       { VecPosition playerPos = getPlayerPosition(playerNum);
        VecPosition ballPos = worldModel->getBall();
        return playerPos.getDistanceTo(ballPos);
    }

// 构造函数，初始化一些变量和参数O
CentralController::CentralController() {
  // 初始化场上信息对象
  fieldInfo = FieldInfo();

  // 初始化策略信息对象
  strategyInfo = StrategyInfo();

  // 初始化当前的策略模式为攻击模式
  strategyInfo.mode =MODE_ATTACK;

  // 初始化所有可用的策略
  strategyInfo.strategies.push_back(Strategy::ATTACK);
  strategyInfo.strategies.push_back(Strategy::DEFEND);
  strategyInfo.strategies.push_back(Strategy::PASS);
  strategyInfo.strategies.push_back(Strategy::SUPPORT);
  strategyInfo.strategies.push_back(Strategy::SHOOT);

  // 初始化每个策略对应的球员编号和优先级为-1，表示未分配
  for (int i = 0; i < NUM_STRATEGIES; i++) {
    strategyInfo.strategyPlayersid[i] = -1;
    strategyInfo.strategyPriorities[i] = -1;
  }

  // 初始化每个球员和每个策略之间的匹配程度矩阵为0，表示未计算
  for (int i = 0; i < NUM_AGENTS; i++) {
    for (int j = 0; j < NUM_STRATEGIES; j++) {
      strategyInfo.matchMatrix[i][j] = 0;
    }
  }
}

// 根据球员的角色和比赛状态来分配策略和优先级
void CentralController::assignStrategyAndPriorityByRoleAndState(WorldModel* worldModel) {
  // 遍历每个球员
  for (int i = 0; i < NUM_AGENTS; i++) {
    // 获取当前球员的编号和角色
    int playerId = strategyInfo.players[i].id;
    Role playerRole = getRole(playerId);

    // 获取当前的比赛状况
    GameState gameState = getGameState(worldModel);

    // 根据不同的角色和比赛状况来分配策略和优先级
    if (playerRole == GOALIE) {
      // 如果是守门员，就根据比赛状态来分配策略和最高优先级
      switch (gameState) {
        case KICK_OFF:
          // 如果是开球，就分配传球策略
          strategyInfo.players[i].strategy = PASS;
          break;
        case DRIBBLE:
          // 如果是带球，就分配防守策略
         strategyInfo.players[i].strategy = DEFEND;
          break;
        case KILL:
          // 如果是射门，就分配防守策略
          strategyInfo.players[i].strategy = DEFEND;
          break;
         case TACKLE:
          // 如果是抢球，就分配防守策略
         strategyInfo.players[i].strategy = DEFEND;
          break;
        default:
          // 其他情况，就分配防守策略
          strategyInfo.players[i].strategy = DEFEND;
          break;
      }
      strategyInfo.players[i].priority = 100;
    }
    else if (playerRole == SUPPORTER) {
      // 如果是支援者，就根据比赛状态来分配策略和中等优先级
      switch (gameState) {
        case KICK_OFF:
          // 如果是开球，就分配支援策略
          strategyInfo.players[i].strategy = SUPPORT;
          break;
        case DRIBBLE:
          // 如果是带球，就分配支援策略
          strategyInfo.players[i].strategy = SUPPORT;
          break;
        case KILL:
          // 如果是射门，就分配传球策略
          strategyInfo.players[i].strategy = PASS;
          break;
        case TACKLE:
          // 如果是抢球，就分配抢球策略
          strategyInfo.players[i].strategy =ATTACK ;
          break;
        default:
          // 其他情况，就分配支援策略
          strategyInfo.players[i].strategy = SUPPORT;
          break;
      }
     strategyInfo.players[i].priority = 50;
    }
    else if (playerRole == BACK_LEFT || playerRole == BACK_RIGHT) {
      // 如果是左后卫或者右后卫，就根据比赛状态来分配策略和低优先级
      switch (gameState) {
        case KICK_OFF:
          // 如果是开球，就分配防守策略
          strategyInfo.players[i].strategy = DEFEND;
          break;
        case DRIBBLE:
          // 如果是带球，就分配防守策略
          strategyInfo.players[i].strategy = DEFEND;
          break;
        case KILL:
          // 如果是射门，就分配防守策略
          strategyInfo.players[i].strategy = DEFEND;
          break;
        case TACKLE:
          // 如果是抢球，就分配抢球策略
          strategyInfo.players[i].strategy = DEFEND;
          break;
        default:
          // 其他情况，就分配防守策略
          strategyInfo.players[i].strategy = DEFEND;
          break;
      }
      strategyInfo.players[i].priority = 10;
    }
    else if (playerRole == MID_LEFT || playerRole == MID_RIGHT) {
      // 如果是左中场或者右中场，就根据比赛状态来分配策略和中等优先级
      switch (gameState) {
        case KICK_OFF:
          // 如果是开球，就分配进攻策略
          strategyInfo.players[i].strategy = ATTACK;
          break;
        case DRIBBLE:
          // 如果是带球，就分配进攻策略
          strategyInfo.players[i].strategy = ATTACK;
          break;
        case KILL:
          // 如果是射门，就分配传球策略
          strategyInfo.players[i].strategy = PASS;
          break;
        case TACKLE:
          // 如果是抢球，就分配抢球策略
          strategyInfo.players[i].strategy = DEFEND;
          break;
        default:
          // 其他情况，就分配进攻策略
          strategyInfo.players[i].strategy = ATTACK;
          break;
      }
      strategyInfo.players[i].priority = 50;
    }
    else if (playerRole == WING_LEFT || playerRole == WING_RIGHT) {
      // 如果是左边锋或者右边锋，就根据比赛状态来分配策略和高优先级
      switch (gameState) {
        case KICK_OFF:
          // 如果是开球，就分配传球策略
          strategyInfo.players[i].strategy = PASS;
          break;
        case DRIBBLE:
          // 如果是带球，就分配传球策略
          strategyInfo.players[i].strategy = PASS;
          break;
        case KILL:
          // 如果是射门，就分配射门策略
          strategyInfo.players[i].strategy = SHOOT;
          break;
        case TACKLE:
          // 如果是抢球，就分配抢球策略
         strategyInfo.players[i].strategy = DEFEND;
          break;
        default:
          // 其他情况，就分配传球策略
          strategyInfo.players[i].strategy = PASS;
          break;
      }
      strategyInfo.players[i].priority = 80;
    }
    else if (playerRole == FORWARD_LEFT || playerRole == FORWARD_RIGHT) {
      // 如果是左前锋或者右前锋，就根据比赛状态来分配策略和高优先级
      switch (gameState) {
        case KICK_OFF:
          // 如果是开球，就分配射门策略
          strategyInfo.players[i].strategy = ATTACK;
          break;
        case DRIBBLE:
          // 如果是带球，就分配射门策略
          strategyInfo.players[i].strategy = ATTACK;
          break;
        case KILL:
        // 如果是射门，就分配射门策略
strategyInfo.players[i].strategy = SHOOT;
break;
case TACKLE:
// 如果是抢球，就分配抢球策略
strategyInfo.players[i].strategy = DEFEND;
break;
default:
// 其他情况，就分配射门策略
strategyInfo.players[i].strategy = SHOOT;
break;
}
strategyInfo.players[i].priority = 80;
}
else {
// 其他情况，就分配默认的策略和优先级
strategyInfo.players[i].strategy = ATTACK;
strategyInfo.players[i].priority = 50;
}
}
//贪心
applyGreedyAlgorithm();
}


// 根据判断条件来调整匹配矩阵
void CentralController::adjustMatchMatrix(WorldModel* worldModel) {
  // 遍历每个球员
  for (int i = 0; i < NUM_AGENTS; i++) {
    // 获取当前球员的编号，位置，速度，角度等信息
    int playerId = strategyInfo.players[i].id;
    VecPosition playerPos = strategyInfo.players[i].pos;
    VecPosition playerVel = strategyInfo.players[i].vel;
    SIM::AngDeg  playerAng = strategyInfo.players[i].ang;
    double distToGoal = HALF_FIELD_X - fieldInfo.ballPos.getX(); //离敌方球门距离
   
   // 根据敌方球员的位置和行为来调整匹配程度，
// 获取敌方球员的数量和信息
// int numOpponents = worldModel->getNumOpponents();
vector<VecPosition> oppPositions = worldModel->getOpponentPositions();
vector<VecPosition> oppVelocities = worldModel->getOpponentVelocities();
vector<SIM::AngDeg> oppAngles = worldModel->getOpponentAngles();

// 遍历每个球员
for (int i = 0; i < NUM_AGENTS; i++) {
  // 获取当前球员的编号，位置，速度，角度等信息
  int playerId = strategyInfo.players[i].id;
  VecPosition playerPos = strategyInfo.players[i].pos;
  VecPosition playerVel = strategyInfo.players[i].vel;
  SIM::AngDeg  playerAng = strategyInfo.players[i].ang;

  // 遍历每个策略
  for (int j = 0; j < NUM_STRATEGIES; j++) {
    // 获取当前策略
    Strategy strategy = static_cast<Strategy>(j);

    // 根据不同的判断条件来调整匹配程度
    if (strategy == ATTACK) {
      // 如果当前策略是进攻，就根据敌方球员到对方球门的距离来调整匹配程度
      // 如果敌方球员离对方球门越远，就提高进攻策略的匹配程度
      double minDistToGoal = HALF_FIELD_X;
      for (int k = 0; k < numOpponents; k++) {
        double distToGoal = HALF_FIELD_X - oppPositions[k].getX();
        if (distToGoal < minDistToGoal) {
          minDistToGoal = distToGoal;
        }
      }
      strategyInfo.matchMatrix[i][ATTACK] += minDistToGoal * 10;
    }
    else if (strategy == DEFEND) {
      // 如果当前策略是防守，就根据敌方球员到己方球门的距离来调整匹配程度
      // 如果敌方球员离己方球门越近，就提高防守策略的匹配程度
      double minDistToGoal = HALF_FIELD_X;
      for (int k = 0; k < numOpponents; k++) {
        double distToGoal = HALF_FIELD_X + oppPositions[k].getX();
        if (distToGoal < minDistToGoal) {
          minDistToGoal = distToGoal;
        }
      }
      strategyInfo.matchMatrix[i][DEFEND] += minDistToGoal * 10;
    }
    else if (strategy == SUPPORT) {
      // 如果当前策略是支援，就根据敌方球员到带球者的距离来调整匹配程度
      // 如果敌方球员离带球者越近，就提高支援策略的匹配程度
      double minDistToBall = HALF_FIELD_X;
      for (int k = 0; k < numOpponents; k++) {
        double distToBall = oppPositions[k].getDistanceTo(worldModel->getBall());
        if (distToBall < minDistToBall) {
          minDistToBall = distToBall;
        }
      }
      strategyInfo.matchMatrix[i][SUPPORT] += minDistToBall * 10;
    }
    else if (strategy == PASS) {
      // 如果当前策略是传球，就根据敌方球员到最佳传球目标的距离来调整匹配程度
      // 如果敌方球员离最佳传球目标越远，就提高传球策略的匹配程度
      double minDistToTarget = HALF_FIELD_X;
      for (int k = 0; k < numOpponents; k++) {
        double distToTarget = oppPositions[k].getDistanceTo(getBestPassTarget());
        if (distToTarget < minDistToTarget) {
          minDistToTarget = distToTarget;
        }
      }
      strategyInfo.matchMatrix[i][PASS] += minDistToTarget * 10;
    }
    else {
      // 其他情况，就保持不变或者使用默认值
     strategyInfo.matchMatrix[i][j] = 50;
    }
  }
}
  }
    }

// 根据贪心算法来优化策略和优先级
void CentralController::applyGreedyAlgorithm() {
  // 定义一个二维数组，用来存储每个策略和球员之间的匹配程度
  double matchArray[NUM_STRATEGIES][NUM_AGENTS];
 // 创建一个存储敌方球员位置的向量
    // std::vector<VecPosition> oppPositions(numOpponents);
    // 获取敌方球员的数量和信息
    // int numOpponents = worldModel->getNumOpponents();
  // 遍历每个球员和每个策略，将匹配程度放入二维数组中
  for (int i = 0; i < NUM_AGENTS; i++) {
    // 获取当前球员的编号
    int playerId = strategyInfo.players[i].id;
    VecPosition currentPosition = strategyInfo.players[i].pos;

    // 遍历每个策略
    for (int j = 0; j < NUM_STRATEGIES; j++) {
      // 获取当前策略
      Strategy strategy = static_cast<Strategy>(j);
      // 如果当前球员已经被分配了策略，就跳过
      if (strategyInfo.strategyPlayersid[i] != -1) {
        continue;
      }
      // 如果当前策略已经被分配了球员，就调整匹配程度
      if (strategyInfo.strategyPlayersid[j] != -1) {
        // 调整公式，根据球员和策略之间的原始匹配程度，距离，角度，速度，优先级，协作程度，竞争程度等因素来调整匹配程度
       strategyInfo.matchMatrix[i][j] =strategyInfo.matchMatrix[i][j] * (1 + distanceFactor + angleFactor + speedFactor + priorityFactor + cooperationFactor - competitionFactor);
      }
      else {
        // 根据不同的判断条件来调整匹配程度
        if (strategy == ATTACK) {
          // 如果当前策略是进攻，就根据敌方球员到对方球门的距离来调整匹配程度
          // 如果敌方球员离对方球门越远，就提高进攻策略的匹配程度
          double minDistToGoal = FIELD_LENGTH / 2;
          for (int k = 0; k < numOpponents; k++) {
          //  oppPositions.push_back(worldModel->getOpponent(k)); // 获取敌方队员的位置并存入向量
            double distToGoal = FIELD_LENGTH / 2 - oppPositions[k].getX();
            if (distToGoal < minDistToGoal) {
              minDistToGoal = distToGoal;
            }
          }
          //根据球员到对方球门的距离来调整匹配程度，同时通过 MATCH_FACTOR 这个常数来控制这个调整的幅度
          //希望球员到对方球门的距离更大程度地影响匹配程度，将 MATCH_FACTOR 设置为一个较大的值1～10
          strategyInfo.matchMatrix[i][ATTACK] += minDistToGoal * MATCH_FACTOR;
        }
        else if (strategy == DEFEND) {
          // 如果当前策略是防守，就根据敌方球员到己方球门的距离来调整匹配程度
          // 如果敌方球员离己方球门越近，就提高防守策略的匹配程度
          double minDistToGoal = FIELD_LENGTH / 2;
          for (int k = 0; k < numOpponents; k++) {
            double distToGoal = FIELD_LENGTH / 2 + oppPositions[k].getX();
            if (distToGoal < minDistToGoal) {
              minDistToGoal = distToGoal;
            }
          }
          strategyInfo.matchMatrix[i][DEFEND] += minDistToGoal * MATCH_FACTOR;
        }
        else if (strategy == SUPPORT) {
          // 如果当前策略是支援，就根据敌方球员到带球者的距离来调整匹配程度
          // 如果敌方球员离带球者越近，就提高支援策略的匹配程度
          double minDistToBall = FIELD_LENGTH / 2;
          for (int k = 0; k < numOpponents; k++) {
            double distToBall = oppPositions[k].getDistanceTo(worldModel->getBall());
            if (distToBall < minDistToBall) {
              minDistToBall = distToBall;
            }
          }
          strategyInfo.matchMatrix[i][SUPPORT] += minDistToBall * MATCH_FACTOR;
        }
        else if (strategy == PASS) {
          // 如果当前策略是传球，就根据敌方球员到最佳传球目标的距离来调整匹配程度
          // 如果敌方球员离最佳传球目标越远，就提高传球策略的匹配程度
          double minDistToTarget = FIELD_LENGTH / 2;
          for (int k = 0; k < numOpponents; k++) {
            double distToTarget = oppPositions[k].getDistanceTo(getBestPassTarget(currentPosition, oppPositions));
            if (distToTarget < minDistToTarget) {
              minDistToTarget = distToTarget;
            }
          }
          strategyInfo.matchMatrix[i][PASS] += minDistToTarget * MATCH_FACTOR;
        }
        else {
          // 其他情况，就保持不变或者使用默认值
          strategyInfo.matchMatrix[i][j] = 5;
        }
      }

      // 将匹配程度放入二维数组中，以便后续选择
      matchArray[j][i] = strategyInfo.matchMatrix[i][j];
    }
  }
//哈希表类似于二维数组但更容易查询
  // 定义一个哈希表，用来存储每个策略和分配给它的球员编号
std::unordered_map<Strategy, std::vector<int>> strategyPlayers;

// 遍历每个策略，找出匹配程度最高的一个或多个球员，并将这些球员分配给这个策略
for (int i = 0; i < NUM_STRATEGIES; i++) {
  // 获取当前策略
  Strategy strategy = static_cast<Strategy>(i);

  // 定义一个变量，用来存储当前策略下的最大匹配程度
  double maxMatchScore = -1;

  // 定义一个向量，用来存储匹配程度最高的一个或多个球员编号
  std::vector<int> bestPlayerIds;

  // 遍历每个球员，找出匹配程度最高的一个或多个球员
  for (int j = 0; j < NUM_AGENTS; j++) {
    // 获取当前球员的编号和匹配程度
    int playerId =strategyInfo.players[j].id;
    double matchScore = matchArray[i][j];

    // 如果当前球员已经被分配了策略，就跳过
    if (strategyInfo.strategyPlayersid[playerId] != -1) {
      continue;
    }

    // 如果当前匹配程度大于最大匹配程度，就更新最大匹配程度，并清空之前的分配结果
    if (matchScore > maxMatchScore) {
      maxMatchScore = matchScore;
      bestPlayerIds.clear();
    }

    // 如果当前匹配程度等于最大匹配程度，就将当前球员加入到分配结果中 
    if (matchScore == maxMatchScore) {
      bestPlayerIds.push_back(playerId);
    }
  }

  // 将选择的策略和球员编号放入哈希表中，以便后续分配
  strategyPlayers[strategy] = bestPlayerIds;
}

// 遍历每个策略和分配给它的球员，将选择的策略分配给这些球员，并给出相应的优先级 
//这里的范围循环语法是 for (auto& pair : strategyPlayers)，其中 auto 是自动类型推断，pair 是一个名字可以随意命名。这个循环会依次将 strategyPlayers 中的每个键值对赋值给 pair，然后你就可以使用 pair.first 和 pair.second 分别访问键和值
for (auto& pair : strategyPlayers) {
  // 获取当前策略和分配给它的球员编号 
  //pair.first 是键（即策略），pair.second 是值（即对应的球员编号向量）
  Strategy strategy = pair.first;
  std::vector<int> playerIds = pair.second;

  // 遍历每个球员，将当前策略分配给他们，并给出相应的优先级
  for (int playerId : playerIds) {
    // 获取当前球员的匹配程度
    double matchScore = matchArray[strategy][playerId];

    // 将选择的策略分配给当前球员，并给出相应的优先级
    strategyInfo.players[playerId].strategy = strategy;
    strategyInfo.players[playerId].priority = matchScore; // 可根据具体需求进行优先级的设定

    // 更新策略信息中的数据结构，记录已经分配的球员和策略
    strategyInfo.strategyPlayersid[playerId] = strategy;
    strategyPlayers[strategy] = playerId;
  }
}

// 遍历其他球员，计算协作程度和冲突程度
for (int i = 0; i < NUM_AGENTS; i++) {
  // 获取当前球员的编号，策略和优先级
  int playerId = strategyInfo.players[i].id;
  Strategy strategy =strategyInfo.players[i].strategy;
  double priority = strategyInfo.players[i].priority;
  Player playerIdall=strategyInfo.players[i];
  // 遍历其他球员，计算协作程度和冲突程度
  for (int j = 0; j < NUM_AGENTS; j++) {
    // 如果是同一个球员，就跳过
    if (i == j) {
      continue;
    }
    // 获取另一个球员的编号，策略和优先级
    int otherPlayerId =strategyInfo.players[j].id;
    Strategy otherStrategy = strategyInfo.players[j].strategy;
    double otherPriority = strategyInfo.players[j].priority;
    Player otherPlayerIdall=strategyInfo.players[i];
    // 如果两个球员使用同一个策略，就提高协作程度，并增加优先级
    if (strategy == otherStrategy) {
      double cooperationDegree = getCooperationDegree(playerIdall, otherPlayerIdall); // 可根据具体需求进行协作程度的计算
      priority += cooperationDegree * PRIORITY_FACTOR;
    }

    // 如果两个球员使用不同的策略，就计算冲突程度，并减少优先级
    else {
      double conflictDegree = getCooperationDegree(playerIdall, otherPlayerIdall); // 可根据具体需求进行冲突程度的计算
      priority -= conflictDegree * PRIORITY_FACTOR;
    }
  }

  // 更新当前球员的优先级
  strategyInfo.players[i].priority = priority;
}
}

// 计算每个球员和每个策略之间的匹配程度，并填充匹配矩阵
void CentralController::calculateMatchMatrix(WorldModel* worldModel) {
  // 定义一些常量和变量
  // const double FIELD_LENGTH = 105.0; // 场地长度
  // const double MAX_SPEED = 8.0; // 最大速度
  const double MAX_PRIORITY = 10.0; // 最大优先级
  const double MAX_COOPERATION = 10.0; // 最大协作程度
  const double MAX_COMPETITION = 10.0; // 最大竞争程度
  // const double MATCH_FACTOR = 10.0; // 匹配因子
  const int NO_PLAYER = -1; // 没有球员的标志

  // 遍历每个球员
  for (int i = 0; i < NUM_AGENTS; i++) {
    // 获取当前球员的编号，位置，速度，角度等信息
  // 获取当前球员的编号，位置，速度，角度等信息
  int playerId = strategyInfo.players[i].id;
  VecPosition playerPos = strategyInfo.players[i].pos;
  VecPosition playerVel = strategyInfo.players[i].vel;
  SIM::AngDeg  playerAng = strategyInfo.players[i].ang;

    // 遍历每个策略
    for (int j = 0; j < NUM_STRATEGIES; j++) {
      // 获取当前策略
      Strategy strategy = static_cast<Strategy>(j);
      // 如果当前球员已经被分配了策略，就跳过
      if (strategyInfo.strategyPlayersid[i] != NO_PLAYER) {
        continue;
      }
      // 如果当前策略已经被分配了球员，就调整匹配程度
      if (strategyInfo.strategyPlayersid[j] != NO_PLAYER) {
        // 调整公式，根据球员和策略之间的原始匹配程度，距离，角度，速度，优先级，协作程度，竞争程度等因素来调整匹配程度
        strategyInfo.matchMatrix[i][j] =  strategyInfo.matchMatrix[i][j] * (1 + distanceFactor + angleFactor + speedFactor + priorityFactor + cooperationFactor - competitionFactor);
      }
      else {
        // 根据不同的判断条件来调整匹配程度
        if (strategy == ATTACK) {
          // 如果当前策略是进攻，就根据敌方球员到对方球门的距离来调整匹配程度
          // 如果敌方球员离对方球门越远，就提高进攻策略的匹配程度
          double minDistToGoal = FIELD_LENGTH / 2;
          for (int k = 0; k < numOpponents; k++) {
            double distToGoal = FIELD_LENGTH / 2 - oppPositions[k].getX();
            if (distToGoal < minDistToGoal) {
              minDistToGoal = distToGoal;
            }
          }
          strategyInfo.matchMatrix[i][ATTACK] += minDistToGoal * MATCH_FACTOR;
        }
        else if (strategy == DEFEND) {
          // 如果当前策略是防守，就根据敌方球员到己方球门的距离来调整匹配程度
          // 如果敌方球员离己方球门越近，就提高防守策略的匹配程度
          double minDistToGoal = FIELD_LENGTH / 2;
          for (int k = 0; k < numOpponents; k++) {
            double distToGoal = FIELD_LENGTH / 2 + oppPositions[k].getX();
            if (distToGoal < minDistToGoal) {
              minDistToGoal = distToGoal;
            }
          }
          strategyInfo.matchMatrix[i][DEFEND] += minDistToGoal * MATCH_FACTOR;
        }
        else if (strategy == SUPPORT) {
          // 如果当前策略是支援，就根据敌方球员到带球者的距离来调整匹配程度
          // 如果敌方球员离带球者越近，就提高支援策略的匹配程度
          double minDistToBall = FIELD_LENGTH / 2;
          for (int k = 0; k < numOpponents; k++) {
            double distToBall = oppPositions[k].getDistanceTo(worldModel->getBall());
            if (distToBall < minDistToBall) {
              minDistToBall = distToBall;
            }
          }
          strategyInfo.matchMatrix[i][SUPPORT] += minDistToBall * MATCH_FACTOR;
        }
        else if (strategy == PASS) {
          // 如果当前策略是传球，就根据敌方球员到最佳传球目标的距离来调整匹配程度
          // 如果敌方球员离最佳传球目标越远，就提高传球策略的匹配程度
          double minDistToTarget = FIELD_LENGTH / 2;
          for (int k = 0; k < numOpponents; k++) {
            double distToTarget = oppPositions[k].getDistanceTo(getBestPassTarget());
            if (distToTarget < minDistToTarget) {
              minDistToTarget = distToTarget;
            }
          }
         strategyInfo.matchMatrix[i][PASS] += minDistToTarget * MATCH_FACTOR;
        }
        else {
          // 其他情况，就保持不变或者使用默认值
          strategyInfo.matchMatrix[i][j] = 5;
        }
      }
    }
  }
}



// 根据球员的编号来返回一个职位
Role CentralController::getRole(int uNum) {
  // 获取每个球员的角色
  map<int, Role> agentRoles = getAgentRoles();
  // 检查是否有对应编号的角色
  if (agentRoles.find(uNum) == agentRoles.end()) {
    // 如果没有，就返回一个默认值
    return GOALIE;
  }
  // 返回对应编号的角色
  return agentRoles[uNum];
}


// 根据比赛状态和球员信息来返回一个比赛状况
GameState CentralController::getGameState(WorldModel* worldModel) {
  
  
  // 获取一些必要的信息，比如球的位置、速度、角度，离球最近的球员等
  VecPosition ballPos = worldModel->getBall();
  VecPosition ballVel = worldModel->getBallVelocity();
  SIM::AngDeg  ballAng = worldModel->getBallDirection();
  int closestPlayer = getPlayerClosestToBall();

  // 根据不同的情况来判断并返回比赛状况
  if (worldModel->getPlayMode() == PM_PLAY_ON) {
    // 如果是开球模式，就返回开球
    return KICK_OFF;
  }
  else if (closestPlayer == worldModel->getUNum()) {
    // 如果离球最近的球员是自己，就判断是带球还是射门
    if (ballPos.getX() > HALF_FIELD_X - 5) {
      // 如果球已经接近对方球门，就返回射门
      return KILL;
    }
    else {
      // 否则，就返回带球
      return DRIBBLE;
    }
  }
  else if (worldModel->getSide() == worldModel->getTeam(closestPlayer)) 
  {
    // 如果离球最近的球员是队友，就返回带球
    return DRIBBLE;
  }
    else
     {
    // 否则，就返回抢球
    return TACKLE;
  }
}

// 切换策略模式，接收一个新的策略模式作为参数
void CentralController::switchStrategyMode(StrategyMode newMode) {
  mode = newMode; // 更新当前的策略模式为新的策略模式
}

void SimpleSoccerBehavior::who_close_to_ball(){
    int OurplayerClosestToBall = -1;
    int OppplayerClosestToBall = -1; //存储距离球最近的队友和对手的球员编号
    double OurclosestDistanceToBall = 10000;
    double OppclosestDistancetoBall = 10000;//存储相应的与球的距离。
    VecPosition posball = worldModel->getBall();

    for (int i=WO_TEAMMATE1; i<WO_TEAMMATE1+NUM_AGENTS; ++i) {//WO_TEAMMATE1=1循环遍历所有球员
        VecPosition temp; 
        int playerNum = i - WO_TEAMMATE1 + 1; //球员编号
        if (worldModel->getUNum()==playerNum) {
            temp = worldModel->getMyPosition();  //temp记录当前队友位置
        } else {
            WorldObject* teammate = worldModel->getWorldObject(i);  //获得i'的worldobjectd对象
            if (teammate->validPosition) //判断球员的位置虽不是对 的
            {
                temp = teammate->pos; //如果是
            } else {
                continue;
            }
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball); //计算当前temp和球的距离
        if (distanceToBall<OurclosestDistanceToBall)//查当前队友与球之间的距离是否小于之前记录的最近队友与球的距离 
         {
            if(!checkingFall()) {    
	      //自注：这里checkingfall的话，可能会存在防守被弹开问题。好处是在前锋的策略里可以避免把摔倒的人看做执行者
	      
                OurplayerClosestToBall = playerNum;
                OurclosestDistanceToBall = distanceToBall;
            }
        }                                                                                                                                                                                                                                             
        who_close_t.our_closest_distance=OurclosestDistanceToBall;//更新最近的球员和距离变量
        who_close_t.our_closest_player=OurplayerClosestToBall;
    }
//和以上代码差不多不过计算的是敌方
    for (int j=WO_OPPONENT1; j<WO_OPPONENT1+NUM_AGENTS; ++j) {
        VecPosition temp;
        int OplayerNUm = j - WO_OPPONENT1 + 1;
        WorldObject* opponent = worldModel->getWorldObject(j);
        if(opponent->validPosition) {
            temp = opponent->pos;
        } else {
            continue;
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall<OppclosestDistancetoBall) {
            OppplayerClosestToBall = OplayerNUm;
            OppclosestDistancetoBall = distanceToBall;
        }
        who_close_t.opp_player_distance=OppclosestDistancetoBall;
        who_close_t.opp_player_player=OppplayerClosestToBall;
    }
}

// 根据世界模型来执行相应的行为
void CentralController::act(WorldModel* worldModel) {
  // 更新场上信息对象
  fieldInfo.update(worldModel);

  // 获取当前的比赛状态
  GameState gameState = getGameState(worldModel);

  // 根据当前的比赛模式和比赛状态来切换策略模式
  switch (strategyInfo.mode) {
    case MODE_ATTACK:
      if (gameState == TACKLE) {
        // 如果当前是进攻模式，但是比赛状态是抢球，就切换到防守模式
        switchStrategyMode(MODE_DEFEND);
      }
      break;
    case MODE_DEFEND:
      if (gameState == DRIBBLE || gameState == KILL) {
        // 如果当前是防守模式，但是比赛状态是带球或者射门，就切换到进攻模式
        switchStrategyMode(MODE_ATTACK );
      }
      break;
    case MODE_PASS:
      if (gameState == TACKLE || gameState ==KILL) {
        // 如果当前是传球模式，但是比赛状态是抢球或者射门，就切换到支援模式
        switchStrategyMode(MODE_SUPPORT);
      }
      break;
    case MODE_SUPPORT:
      if (gameState == DRIBBLE || gameState == KILL) {
        // 如果当前是支援模式，但是比赛状态是带球或者射门，就切换到射门模式
        switchStrategyMode(MODE_SHOOT);
      }
      break;
  }

  // 计算每个球员和每个策略之间的匹配程度，并填充匹配矩阵
  calculateMatchMatrix(worldModel);

  // 根据贪心算法来优化策略和优先级
  applyGreedyAlgorithm();

  // 遍历每个球员，执行相应的策略
  for (int i = 0; i < NUM_AGENTS; i++) {
    // 获取当前球员的编号和策略
    int playerId =strategyInfo.players[i].id;
    Strategy strategy =strategyInfo. players[i].strategy;

    // 根据不同的策略来执行不同的行为
    switch (strategy) {
      case ATTACK:
        // 执行进攻行为
        attack(playerId, worldModel);
        break;
      case DEFEND:
        // 执行防守行为
        defend(playerId, worldModel);
        break;
      case PASS:
        // 执行传球行为
        pass(playerId, worldModel);
        break;
      case SHOOT:
        // 执行射门行为
        shoot(playerId, worldModel);
        break;
      case SUPPORT:
        // 执行支援行为
        support(playerId, worldModel);
        break;
    }
  }
}
 // 更新敌方球员位置信息的函数
    void CentralController::updateOpponentPositions(WorldModel* worldModel) {
        for (int k = 0; k < NUM_AGENTS; k++) {
            oppPositions[k] = worldModel->getOpponent(k); // 获取第 k 个敌方队员的位置
        }
    }

 
double CentralController::getCooperationDegree(Player player1, Player player2) {
  // 获取两个球员的位置和策略
  VecPosition position1 = player1.pos;
  VecPosition position2 = player2.pos;
  Strategy strategy1 = player1.strategy;
  Strategy strategy2 = player2.strategy;

  // 定义一个变量，用来存储协作程度
  double cooperationDegree = 0;
 //初始化贡献值
   double contribution1=0;
 double contribution2=0;
  // 如果两个球员使用同一个策略，就根据他们到策略目标的距离和角度来计算协作程度
  // 如果两个球员到策略目标的距离越近，且他们之间的角度越小（即越平行），就提高协作程度
  if (strategy1 == strategy2) {
    // 获取策略目标
    VecPosition strategyTarget =getTarget(strategy1);

    // 计算两个球员到策略目标的距离
    double distance1 = position1.getDistanceTo(strategyTarget);
    double distance2 = position2.getDistanceTo(strategyTarget);

    // 计算两个球员之间的角度
    double angle = getAngleBetween(position2,position2);

    // 根据距离和角度来计算协作程度
    cooperationDegree = (FIELD_LENGTH - distance1 - distance2) / FIELD_LENGTH * cos(angle);
  }

  // 如果两个球员使用不同的策略，就根据他们对策略目标的贡献来计算协作程度
  // 如果两个球员对策略目标的贡献越高（即越接近1），就提高协作程度
 
  else {
      if (strategy1=ATTACK)
    { 
       // 计算1号球员对策略目标的贡献
     contribution1 = getContributionToShoot(player1);
    }
    else if(strategy1=PASS)
    {
    contribution1 = getContributionToPass(player1);
    } 
    // 根据贡献来计算协作程度
    else
    {
     contribution1 = getContributionToShoot(player1);       
    } 
  if (strategy2=ATTACK)
  {
     // 计算2号球员对策略目标的贡献
    contribution2 = getContributionToShoot(player2);
  }
  else if(strategy2=PASS)
  {
    contribution2 = getContributionToPass(player2);
  }
  else{
      contribution2 = getContributionToShoot(player2);
  }
    cooperationDegree = (contribution1 + contribution2) / 2;
  }
  // 返回协作程度
  return cooperationDegree;
}

// 定义一个函数，根据策略返回目标位置
VecPosition CentralController::getTarget(Strategy strategy) {
  switch (strategy) {
    case ATTACK: return fieldInfo.ballPos; // 如果策略是进攻，目标就是球位置
    case DEFEND: return VecPosition(-FIELD_LENGTH / 2, 0); // 如果策略是防守，目标就是己方球门
    case PASS: return getBestPassTarget(); // 如果策略是传球，目标就是最佳传球目标
    case SHOOT: return VecPosition(FIELD_LENGTH / 2, 0); // 如果策略是射门，目标就是对方球门
    //case SUPPORT: return getBestSupportPosition(); // 如果策略是支援，目标就是最佳支援位置
  }
}

 // 定义一个函数返回两个位置之间的角度
    //inline double getAngleBetweenPoints( const VecPosition &p1,
double CentralController::getAngleBetween(const VecPosition& position1, const VecPosition& position2) {
    double deltaY = position2.getY() - position1.getY();
    double deltaX = position2.getX() - position1.getX();
    
    // 使用 atan2 函数来计算角度，返回的角度是弧度值
    double angleRadians = atan2(deltaY, deltaX);
    
    // 将弧度值转换为角度值并返回
    double angleDegrees = angleRadians * 180.0 / M_PI;
    
    return angleDegrees;
}

// // 定义一个函数，计算一个球员对传球策略目标的贡献
// double  CentralController::getContributionToPass(Player playerC) {
//   // 获取当前球员和策略目标的位置、速度、角度等信息
//   VecPosition playerPos = playerC.pos;
//   VecPosition playerVel = playerC.vel;
//   SIM::AngDeg playerAng = playerC.ang;
//   VecPosition targetPos = getBestPassTarget(); // 获取最佳传球目标的位置
//   VecPosition targetVel = targetPos.getVelocity(); // 获取最佳传球目标的速度
//   SIM::AngDeg targetAng = targetPos. getAngleBetweenPoints(); // 获取最佳传球目标的角度

//   // 计算一些相关的指标
//   double distance = playerPos.getDistanceTo(targetPos); // 球员到目标的距离
//   double angle = playerPos.getAngleBetweenPoints(playerPos,targetPos); // 球员到目标的角度
//   double speed = playerVel.getMagnitude(); // 球员的速度
//   double time = distance / speed; // 球员到达目标所需的时间

//   // 计算一个加权平均值
//   double weightDistance = -1.0; // 距离越近，贡献越高
//   double weightAngle = -1.0; // 角度越小，贡献越高
//   double weightSpeed = 1.0; // 速度越快，贡献越高
//   double weightTime = -1.0; // 时间越短，贡献越高
//   double weightedAverage = weightDistance * distance + weightAngle * angle + weightSpeed * speed + weightTime * time;

//   // 将加权平均值归一化到0到1之间
//   double minWeightedAverage = -FIELD_LENGTH - FIELD_WIDTH - MAX_SPEED - FIELD_LENGTH / MAX_SPEED; // 最小可能的加权平均值
//   double maxWeightedAverage = FIELD_LENGTH + FIELD_WIDTH + MAX_SPEED + FIELD_LENGTH / MAX_SPEED; // 最大可能的加权平均值
//   double normalizedWeightedAverage = (weightedAverage - minWeightedAverage) / (maxWeightedAverage - minWeightedAverage);

//   // 返回最终的贡献值
//   return normalizedWeightedAverage;
// }
// 定义一个函数，计算一个球员对传球策略目标的贡献
double CentralController::getContributionToPass(Player playerC) {
  // 获取当前球员和策略目标的位置、速度、角度等信息
  VecPosition playerPos = playerC.pos;
  VecPosition playerVel = playerC.vel;
  SIM::AngDeg playerAng = playerC.ang;
  VecPosition targetPos = getBestPassTarget(); // 获取最佳传球目标的位置

  // 计算一些相关的指标
  double distance = playerPos.getDistanceTo(targetPos); // 球员到目标的距离
  double angle = playerPos.getAngleBetweenPoints(playerPos, targetPos); // 球员到目标的角度
  double speed = playerVel.getMagnitude(); // 球员的速度
  double time = distance / speed; // 球员到达目标所需的时间

  // 计算一个加权平均值
  double weightDistance = -1.0; // 距离越近，贡献越高
  double weightAngle = -1.0; // 角度越小，贡献越高
  double weightSpeed = 1.0; // 速度越快，贡献越高
  double weightTime = -1.0; // 时间越短，贡献越高
  double weightedAverage = weightDistance * distance + weightAngle * angle + weightSpeed * speed + weightTime * time;

  // 将加权平均值归一化到0到1之间
  double minWeightedAverage = -FIELD_LENGTH - FIELD_WIDTH - MAX_SPEED - FIELD_LENGTH / MAX_SPEED; // 最小可能的加权平均值
  double maxWeightedAverage = FIELD_LENGTH + FIELD_WIDTH + MAX_SPEED + FIELD_LENGTH / MAX_SPEED; // 最大可能的加权平均值
  double normalizedWeightedAverage = (weightedAverage - minWeightedAverage) / (maxWeightedAverage - minWeightedAverage);

  // 返回最终的贡献值
  return normalizedWeightedAverage;
}

// 定义一个函数，计算一个球员对射门策略目标的贡献
double getContributionToShoot(Player playerC) {
  // 获取当前球员和策略目标的位置、速度、角度等信息
  VecPosition playerPos = playerC.pos;
  VecPosition playerVel = playerC.vel;
  SIM::AngDeg playerAng = playerC.ang;
  VecPosition targetPos = VecPosition(FIELD_LENGTH / 2, 0);
  VecPosition targetVel = VecPosition(0, 0);
  SIM::AngDeg targetAng = 0;

  // 计算一些相关的指标
  double distance = playerPos.getDistanceTo(targetPos); // 球员到目标的距离
  double angle = playerPos.getAngleBetweenPoints(playerPos,targetPos); // 球员到目标的角度
  double speed = playerVel.getMagnitude(); // 球员的速度
  double time = distance / speed; // 球员到达目标所需的时间

  // 计算一个加权平均值
  double weightDistance = -1.0; // 距离越近，贡献越高
  double weightAngle = -1.0; // 角度越小，贡献越高
  double weightSpeed = 1.0; // 速度越快，贡献越高
  double weightTime = -1.0; // 时间越短，贡献越高
  double weightedAverage = weightDistance * distance + weightAngle * angle + weightSpeed * speed + weightTime * time;

  // 将加权平均值归一化到0到1之间
  double minWeightedAverage = -FIELD_LENGTH - FIELD_WIDTH - MAX_SPEED - FIELD_LENGTH / MAX_SPEED; // 最小可能的加权平均值
  double maxWeightedAverage = FIELD_LENGTH + FIELD_WIDTH + MAX_SPEED + FIELD_LENGTH / MAX_SPEED; // 最大可能的加权平均值
  double normalizedWeightedAverage = (weightedAverage - minWeightedAverage) / (maxWeightedAverage - minWeightedAverage);

  // 返回最终的贡献值
  return normalizedWeightedAverage;
}
//传球
// 定义一个函数，计算一个候选目标的得分
double calculateScore(PassTarget target, FieldInfo fieldInfo)
{
  double score = 0.0; // 初始化得分为0
  // 计算候选目标距离球的距离
  double distToBall = target.pos.getDistanceTo(fieldInfo.ballPos);
  // 计算候选目标距离对方球员最近的距离
  double distToOpp = DBL_MAX; // 初始化为最大值
  for (int i = 0; i < fieldInfo.enemyPlayers.size(); i++)
  {
    double dist = target.pos.getDistanceTo(fieldInfo.enemyPlayers[i].pos);
    if (dist < distToOpp)
    {
      distToOpp = dist;
    }
  }
  // 计算候选目标距离己方球门中心的距离
  double distToGoal = target.pos.getDistanceTo(VecPosition(0, -FIELD_LENGTH / 2.0));
  // 计算候选目标与球和己方球门中心形成的夹角
 SIM::AngDeg angToGoal = (target.pos - fieldInfo.ballPos).getAngleWithVector(VecPosition(0, -FIELD_LENGTH / 2.0) - fieldInfo.ballPos);
  // 根据不同的因素给候选目标打分，可以根据需要调整权重
  score += (1.0 / distToBall) * 10.0; // 距离球越近，得分越高
  score += (distToOpp / FIELD_LENGTH) * 10.0; // 距离对方球员越远，得分越高
  score += (1.0 / distToGoal) * 5.0; // 距离己方球门越远，得分越高
  score += (angToGoal / M_PI) * 5.0; // 夹角越大，得分越高
  return score;
}

// 定义一个函数，找出所有合法的传球候选目标，并将它们存入一个向量中
vector<PassTarget>  CentralController::findPassTargets(FieldInfo fieldInfo)
{
  vector<PassTarget> targets; // 初始化一个空向量
  for (int i = 0; i < fieldInfo.players.size(); i++)
  {
    Player player = fieldInfo.players[i]; // 取出第i个球员
    if (player.id != fieldInfo.onBallPlayerId) // 如果该球员不是拥有球权的球员，则可以作为传球候选目标
    {
      PassTarget target; // 创建一个传球候选目标对象
      target.id = player.id; // 设置其编号为该球员的编号
      target.pos = player.pos; // 设置其位置为该球员的位置
      target.ang=player.pos.getAngleBetweenPoints(player.pos, fieldInfo.ballPos);
      target.score = calculateScore(target, fieldInfo); // 调用函数计算其得分
      targets.push_back(target); // 将该候选目标加入向量中
    }
  }
  return targets;
}

// 定义一个函数，从候选目标向量中选择得分最高的一个，并返回其位置
VecPosition CentralController::getBestPassTarget(FieldInfo fieldInfo)
{
  vector<PassTarget> targets = findPassTargets(fieldInfo); // 调用函数找出所有合法的传球候选目标
  PassTarget bestTarget; // 初始化一个最佳传球候选目标对象
  bestTarget.score = -DBL_MAX; // 初始化其得分为最小值
  for (int i = 0; i < targets.size(); i++)
  {
    PassTarget target = targets[i]; // 取出第i个候选目标
    if (target.score > bestTarget.score) // 如果该候选目标的得分高于当前最佳候选目标的得分，则更新最佳候选目标
    {
      bestTarget = target;
    }
  }
  return bestTarget.pos; // 返回最佳传球候选目标的位置
}

void FieldInfo::update(WorldModel *worldModel) {
  ballPos = worldModel->getBall();
  ballVel = worldModel->getBallVelocity();
  ballAng = worldModel->getBallDirection();

  // 更新所有球员的信息
  players.clear();
  enemyPlayers.clear();
  for (int i = 0; i < NUM_AGENTS; i++) {
    Player player;
    player.id = i + 1;
    player.pos = worldModel->GetMyPosition(i);
    player.vel = worldModel->GetSpeed(i);
    player.ang = worldModel->GetMyAngle(i);
    // 这里根据需要设置其他球员属性
    players.push_back(player);
    
    // 根据球员编号和是否为敌方球员，判断是否将球员信息加入敌方球员列表
    if (player.id >= WO_OPPONENT1 && player.id <= WO_OPPONENT1 + NUM_AGENTS - 1) {
      enemyPlayers.push_back(player);
    }
  }

  // 设置拥有球权的球员编号
  onBallPlayerId = worldModel->getUNum();
}
