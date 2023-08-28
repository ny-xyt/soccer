#ifndef _SIMPLE_SOCCER_H
#define _SIMPLE_SOCCER_H
#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"
#include "../utwalk/math/Vector.h"
#include "../utwalk/math/Vector3.h"
#include "Vector3f.h"
#define FIELD_LENGTH 11
#define MATCH_FACTOR 2 // 匹配因子
#define FIELD_WIDTH 15
#define MAX_SPEED  8.0
#define PRIORITY_FACTOR  1
enum Role
{                // 枚举了一些身份
  GOALIE,        // 守门员
  SUPPORTER,     // 职员者
  BACK_LEFT,     // 左后卫
  BACK_RIGHT,    // 右后卫
  MID_LEFT,      // 左中场
  MID_RIGHT,     // 右中场
  WING_LEFT,     // 左边锋
  WING_RIGHT,    // 右边锋
  FORWARD_LEFT,  // 左前锋
  FORWARD_RIGHT, // 右前锋
  ON_BALL,       // 控球
  NUM_ROLES
};
struct who_close
{
  float our_closest_player;
  float our_closest_distance;
  float opp_player_player;
  float opp_player_distance;
};
// 定义一个类 SimpleSoccerBehavior,他是NaoBehavior的子类
class SimpleSoccerBehavior : public NaoBehavior
{
public:
  // 初始化NaoBehavior
  SimpleSoccerBehavior(const std::string teamName, int uNum, const map<string, string> &namedParams_, const string &rsg_);
  virtual void beam(double &beamX, double &beamY, double &beamAngle);
  virtual SkillType selectSkill();
  void who_close_to_ball();

protected:
  int getPlayerClosestToBall();                       // 返回离球最近的球员编号
  SkillType goToSpace(VecPosition space);             // 让球员跑向一个空位
  SkillType watchPosition(VecPosition pos);           // 让球员注视一个位置
  VecPosition getSpaceForRole(Role role);             // 根据角色返回一个空位
  map<int, Role> getAgentRoles();                     ////用来返回每个球员的角色的
  double getAgentDistanceToRole(int uNum, Role role); // 返回一个球员到一个角色 的距离
};

// 定义一个枚举类型，表示不同的比赛状况
enum GameState
{
  KICK_OFF, // 开球
  DRIBBLE,  // 带球
  KILL,     // 射门
  TACKLE    // 抢球
};
// 定义一个枚举类型，比赛模式
enum StrategyMode
{
  MODE_ATTACK,  // 进攻
  MODE_DEFEND,  // 防守
  MODE_SUPPORT, // 支援
  MODE_PASS,    // 传球
  MODE_SHOOT    // 射门
};
// 定义一个枚举类型，表示不同的策略模式,比赛策略
enum Strategy
{
  ATTACK, // 进攻
  DEFEND, // 防守
  PASS,   // 传球
  SHOOT,  // 射门
  SUPPORT // 支援

};

// 定义一个枚举类型，表示不同的具体分类
enum Category
{
  KICK_OFF_1,      // 开球1
  KICK_OFF_2,      // 开球2
  DRIBBLE_FORWARD, // 向前带球
  DRIBBLE_PASS,    // 传球
  DRIBBLE_SUPPORT, // 支援带球者
  DRIBBLE_DEFEND,  // 防守带球者
  DRIBBLE_ATTACK,  // 进攻带球者
  DRIBBLE_WING,    // 边路带球者
  SHOOT_LONG,      // 远程射门
  SHOOT_SHORT,     // 近距离射门
  SHOOT_PASS,      // 射门前传球
  SHOOT_DEFEND,    // 防守射门者
  SHOOT_CROSS,     // 横传射门者
  SHOOT_SHOT,      // 直接射门者
  TACKLE_TACKLE,   // 劫球者
  TACKLE_DEFEND    // 防守劫球者
};

// 定义一个结构体，表示每个球员的信息
struct Player
{
  int id;            // 编号
  VecPosition pos;   // 位置
  VecPosition vel;   // 速度
  SIM::AngDeg ang;   // 角度
  Strategy strategy; // 策略
  int priority;      // 优先级
};
// 定义一个结构体，表示场上信息
struct FieldInfo
{
  // 场上信息的属性
  VecPosition ballPos;         // 球的位置
  VecPosition ballVel;         // 球的速度
  SIM::AngDeg ballAng;         // 球的角度
  vector<Player> players;      // 所有球员的信息
  vector<Player> enemyPlayers; // 敌方球员的信息
  int onBallPlayerId;          // 拥有球权的球员编号
  // 场上信息的方法
  FieldInfo();                         // 构造函数，初始化场上信息
  void update(WorldModel *worldModel); // 根据世界模型更新场上信息
};

// 定义一个结构体，表示策略信息
struct StrategyInfo
{
  StrategyMode mode;                              // 策略模式
  vector<Strategy> strategies;                    // 所有可用的策略
  vector<int> strategyPlayersid;                  // 每个策略对应的球员编号
  vector<double> strategyPriorities;              // 每个策略对应的优先级
  double matchMatrix[NUM_AGENTS][NUM_STRATEGIES]; // 每个球员和每个策略之间的匹配程度矩阵
  vector<Player> players;                         // 所有球员的信息
};
 // 定义一个结构体，表示传球的候选目标
struct PassTarget
{
  int id; // 候选目标的编号
  VecPosition pos; // 候选目标的位置
  double score; // 候选目标的得分
  SIM::AngDeg ang; // 候选目标的角度
};

// 定义一个中央控制器类，用来管理所有的策略和行为
class CentralController
{
public:
// 定义一个函数，从候选目标向量中选择得分最高的一个，并返回其位置
VecPosition getBestPassTarget(FieldInfo fieldInfo);
vector<PassTarget>  findPassTargets(FieldInfo fieldInfo);
// 定义一个函数，根据策略返回最佳传球目标
VecPosition getBestPassTarget();
// 定义一个函数，计算一个球员对传球策略目标的贡献
double getContributionToPass(Player playerC);
 //定义一个函数返回2个位置之间的角度
  double getAngleBetween(const VecPosition& position1, const VecPosition& position2);
// 定义一个函数，根据策略返回目标位置
  VecPosition getTarget(Strategy strategy);
  double getCooperationDegree(Player player1, Player player2);
  void updateOpponentPositions(WorldModel *worldModel);
  void CentralController::adjustMatchMatrix(WorldModel *worldModel);
  map<int, Role> getAgentRoles(); // 给每个机器人分配一个角色
  // 根据球员的编号来返回一个职位
  Role CentralController::getRole(int uNum);
  void assignStrategyAndPriorityByRoleAndState(WorldModel *worldModel);
  CentralController();                           // 构造函数，初始化一些变量和参数
  void act(WorldModel *worldModel);              // 根据世界模型来执行相应的行为
  void switchStrategyMode(StrategyMode newMode); // 切换策略模式，接收一个新的策略模式作为参数
  int getPlayerClosestToBall();

private:
  FieldInfo fieldInfo;       // 场上信息对象
  StrategyInfo strategyInfo; // 策略信息对象
                             // 定义一个数组来存储敌方球员位置信息
  VecPosition oppPositions[NUM_AGENTS];
  GameState getGameState(WorldModel *worldModel);       // 根据场上信息和球员信息来返回一个比赛状态
  Role getRole(int playerId, GameState gameState);      // 根据球员编号和比赛状态来返回一个角色身份
  Category getCategory(Role role, GameState gameState); // 根据角色身份和比赛状态来返回一个具体分类
  void calculateMatchMatrix(WorldModel *worldModel);    // 计算每个球员和每个策略之间的匹配程度，并填充匹配矩阵
  void applyGreedyAlgorithm();                          // 根据贪心算法来优化策略和优先级
  Strategy getBestStrategy(int playerId);               // 根据球员编号返回最优的策略
  // 定义权重因素
  double distanceFactor = 1.0;
  double angleFactor = 1.0;
  double speedFactor = 1.0;
  double priorityFactor = 1.0;
  double cooperationFactor = 1.0;
  double competitionFactor = 1.0;
};
#endif
