#ifndef WORLDMODEL_H
#define WORLDMODEL_H

#include "../headers/headers.h"
#include "../math/Geometry.h"
#include "../math/hctmatrix.h"
#include "WorldObject.h"
#include "../headers/Field.h"
#include "../rvdraw/rvdraw.h"

class BallKF;
class PlayerKF;

using namespace std;
typedef enum {LEFT,RIGHT}side;
class WorldModel {
//存放和更新湖区日对于场地和其他对象的信息
private:
    
//   // 敌方球员的数量
//     int numOpponents;
    VecPosition positions[NUM_AGENTS]; // the positions of all agents
    SIM::AngDeg angles[NUM_AGENTS]; // the angles of all agents
    VecPosition speeds[NUM_AGENTS]; // the speeds of all agents
    unsigned long cycle; //表示当前周期数
    int scoreLeft;     //左边分
    int scoreRight;
    double time;      //时间
    double gameTime;   //比赛进行时间
    int playMode;  //当前比赛模式
    int lastPlayMode;   //上一个比赛模式
    int lastDifferentPlayMode;   //
    int uNum;      //机器人编号
    int Side;   //在左边右边
    static const int MAX_TEAMMATES = NUM_AGENTS + 1; // 队友的最大数量
    //side team;  //机器人所在队伍
    bool uNumSet;   
    bool sideSet;
   // 成员变量用于存储敌方球员位置信息
    std::vector<VecPosition> oppPositions;
    WorldObject worldObjects[NUM_WORLD_OBJS];  //存放场上所有的对象信息
    VecPosition myPosition; //存储了机器人自己的位置
    SIM::AngDeg myAngDegrees;//存储机器人自己角度
    bool confident;  //机器人对自己位置角度的相信度
 
    VecPosition myLastPosition;  //机器人上一次的位置
    SIM::AngDeg myLastAngDegrees;  //机器人上次的角度
    RVSender *rvsend;       //
    bool fUseGroundTruthDataForLocalization;
    // TODO: comment it out if we don't want ground truth
#define GROUND_TRUTH_SERVER      
#ifdef GROUND_TRUTH_SERVER
    VecPosition myPositionGroundTruth;      //存储机器人的真实位置
    SIM::AngDeg myAngGroundTruth;   ////真实角度
    VecPosition ballGroundTruth;       //球的真实位置
#endif

    double lastBallSightingTime;  //存储机器人上一次看到球的时间
    double lastLineSightingTime;         //存储机器人上一次看到线的时间

    // This is where we actually saw the ball ourself
    vector<VecPosition> lastBallSeenPosition;
    vector<double> lastBallSeenTime;

    // remember last two skills
    vector<SkillType> lastSkills;
    vector<SkillType> executedSkillsForOdometry;

    // record last odometry from particle filter
    SIM::Point2D lastOdometryPos;
    double lastOdometryAngDeg;

    bool fLocalized;
    bool fFallen;

    HCTMatrix localToGlobal, globalToLocal;

    vector<bool> fallenTeammate;
    vector<bool> fallenOpponent;

    string opponentTeamName;

    BallKF* ballKalmanFilter;
    PlayerKF* opponentKalmanFilters;

    double teammateLastHeardTime[NUM_AGENTS];

public:

    WorldModel();
    ~WorldModel();


     inline VecPosition getOpponentPosition(const int &i) const {
        return worldObjects[i].pos;
    }

    inline SIM::AngDeg getOpponentAngle(const int &i) const {
        return worldObjects[i].ang;
    }


    inline void setMyConfidence(bool confidence) {
        confident = confidence;
    }
    inline bool getMyConfidence() const {
        return confident;
    }
 

    // 添加一个成员函数来更新敌方球员的位置信息
    void updateOpponentPositions() {
        for (int k = 0; k < NUM_AGENTS; k++) {
            oppPositions[k] = getOpponent(k); // 获取第 k 个敌方队员的位置
        }
    }

    inline WorldObject* getWorldObject( int index ) {
        return &worldObjects[index];
    }
    void updateGoalPostsAndFlags();
    void updateMatricesAndMovingObjs( VecPosition& fieldXPlusYPlus,
                                      VecPosition& fieldXPlusYMinus,
                                      VecPosition& fieldXMinusYPlus,
                                      VecPosition& fieldXMinusYMinus );


    inline void setMyPosition( const VecPosition& newPos ) {
        myPosition = newPos;
    }
    inline void setMyPosition( const SIM::Point2D& newPos ) {
        myPosition.setX( newPos.getX() );
        myPosition.setY( newPos.getY() );
        // Z is not changed, stays at the default
    }
    inline VecPosition getMyPosition() const {
        return myPosition;
    }
   inline VecPosition GetMyPosition(int id) const {
        return positions[id];
    }
// 获取球的速度向量
inline VecPosition getBallVelocity() const {
  return worldObjects[WO_BALL].vel;
}
// 获取球的方向角
inline SIM::AngDeg getBallDirection() const {
  return worldObjects[WO_BALL].ang;
}

inline side WorldModel::getTeam(int uNum)
{
  return worldObjects[uNum].Side;
}
    inline SIM::AngDeg GetMyAngle(int id) const {
        return angles[id];
    }
    inline VecPosition GetSpeed(int id) const {
        return speeds[id];
    }
    inline void setMyAngDeg( SIM::AngDeg newAng ) {
        myAngDegrees = newAng;
    }
    inline void setMyAngRad( SIM::AngRad newAng ) {
        myAngDegrees = Rad2Deg( newAng );
    }
    inline SIM::AngDeg getMyAngDeg() const {
        return myAngDegrees;
    }
    inline SIM::AngRad getMyAngRad() const {
        return Deg2Rad( myAngDegrees );
    }


    inline void setMyLastPosition( const VecPosition& newPos ) {
        myLastPosition = newPos;
    }
    inline VecPosition getMyLastPosition() const {
        return myLastPosition;
    }


    inline void setMyLastAngDeg( SIM::AngDeg newAng ) {
        myLastAngDegrees = newAng;
    }
    inline void setMyLastAngRad( SIM::AngRad newAng ) {
        myLastAngDegrees = Rad2Deg( newAng );
    }
    inline SIM::AngDeg getMyLastAngDeg() const {
        return myLastAngDegrees;
    }
    inline SIM::AngRad getMyLastAngRad() const {
        return Deg2Rad( myLastAngDegrees );
    }

    inline RVSender *getRVSender() const {
        return rvsend;
    }

    inline void setUseGroundTruthDataForLocalization(bool fUseGroundTruthDataForLocalization) {
        this->fUseGroundTruthDataForLocalization = fUseGroundTruthDataForLocalization;
    }
    inline bool useGroundTruthDataForLocalization() {
        return fUseGroundTruthDataForLocalization;
    }

#ifdef GROUND_TRUTH_SERVER
    inline void setMyPositionGroundTruth( const VecPosition& newPos ) {
        myPositionGroundTruth = newPos;
    }
    inline VecPosition getMyPositionGroundTruth() const {
        return myPositionGroundTruth;
    }

    inline void setMyAngDegGroundTruth( double angDeg ) {
        myAngGroundTruth = angDeg;
    }
    inline double getMyAngDegGroundTruth() const {
        return myAngGroundTruth;
    }

    inline void setBallGroundTruth( VecPosition newBall ) {
        ballGroundTruth = newBall;
    }
    inline VecPosition getBallGroundTruth() const {
        return ballGroundTruth;
    }
#endif

    inline void setLastBallSightingTime(double lastTime) {
        lastBallSightingTime = lastTime;
    }
    inline double getLastBallSightingTime() const {
        return lastBallSightingTime;
    }

    inline void setLastLineSightingTime(double lastTime) {
        lastLineSightingTime = lastTime;
    }
    inline double getLastLineSightingTime() const {
        return lastLineSightingTime;
    }

    inline void setLastBallSeenPosition(VecPosition position) {
        lastBallSeenPosition.insert(lastBallSeenPosition.begin(),position);
        lastBallSeenPosition.pop_back();
    }
    inline vector<VecPosition> getLastBallSeenPosition() const {
        return lastBallSeenPosition;
    }

    inline void setLastBallSeenTime(double time) {
        lastBallSeenTime.insert(lastBallSeenTime.begin(),time);
        lastBallSeenTime.pop_back();
    }

    inline vector<double> getLastBallSeenTime() const {
        return lastBallSeenTime;
    }

    inline unsigned long getCycle() const {
        return cycle;
    }
    inline void incrementCycle() {
        cycle++;
    }
    inline void setCycle(const unsigned long &cycle) {
        this->cycle = cycle;
    }

    inline int getScoreLeft() const {
        return scoreLeft;
    }
    inline void setScoreLeft(const int &scoreLeft) {
        this->scoreLeft = scoreLeft;
    }

    inline int getScoreRight() const {
        return scoreRight;
    }
    inline void setScoreRight(const int &scoreRight) {
        this->scoreRight = scoreRight;
    }

    inline double getTime() const {
        return time;
    }
    inline void setTime(const double &time) {
        this->time = time;
    }

    inline double getGameTime() const {
        return gameTime;
    }
    inline void setGameTime(const double &gameTime) {
        this->gameTime = gameTime;
    }

    inline int getPlayMode() const {
        return playMode;
    }
    inline void setPlayMode(const int &playMode) {
        this->playMode = playMode;
    }

    inline int getLastPlayMode() const {
        return lastPlayMode;
    }
    inline void setLastPlayMode(const int &lastPlayMode) {
        this->lastPlayMode = lastPlayMode;
    }

    inline int getLastDifferentPlayMode() const {
        return lastDifferentPlayMode;
    }
    inline void setLastDifferentPlayMode(const int &lastDifferentPlayMode) {
        this->lastDifferentPlayMode = lastDifferentPlayMode;
    }

    inline int getUNum() const {
        return uNum;
    }
    inline void setUNum(const int &uNum) {
        this->uNum = uNum;
    }

    inline bool getUNumSet() const {
        return uNumSet;
    }
    inline void setUNumSet(const bool &uNumSet) {
        this->uNumSet = uNumSet;
    }


    inline SkillType getLastSkill() const {
        return lastSkills[0];
    }
    inline SkillType getPreviousLastSkill() const {
        return lastSkills[1];
    }
    inline void setLastSkill(const SkillType &lastSkill) {
        this->lastSkills[1] = this->lastSkills[0];
        this->lastSkills[0] = lastSkill;
    }



    // functions for odometry
    inline void addExecutedSkill(const SkillType &skill) {
        executedSkillsForOdometry.push_back( skill );
    }
    inline const vector<SkillType>& getExecutedSkills() const {
        return executedSkillsForOdometry;
    }
    inline void resetExecutedSkills() {
        executedSkillsForOdometry.clear();
    }

    // tracking odometry
    inline SIM::Point2D& getLastOdometryPos() {
        return lastOdometryPos;
    }
    inline void setLastOdometryPos(const SIM::Point2D& pos) {
        lastOdometryPos = pos;
    }

    inline double& getLastOdometryAngDeg() {
        return lastOdometryAngDeg;
    }
    inline void setLastOdometryAngDeg(const double& ang) {
        lastOdometryAngDeg = ang;
    }

    // THIS IS A SINGLE POINT DETERMINES WHETHER USING BALL KALMAN FILTER
    inline bool useKalmanFilter() {
        return true;
    }


    inline int getSide() {
        return Side;
    }
    inline void setSide(const int &side) {
        this->Side = side;
        updateGoalPostsAndFlags();
    }

    inline bool getSideSet() {
        return sideSet;
    }
    inline void setSideSet(const bool &sideSet) {
        this->sideSet = sideSet;
    }

    inline VecPosition getGoalPost(const int &i) const {
        return worldObjects[GOALPOST_1_L  + i].pos;
    };

    inline VecPosition getMyLeftGoalPost() {
        return ((Side == SIDE_LEFT)? worldObjects[GOALPOST_1_L].pos : worldObjects[GOALPOST_2_R].pos );
    }
    inline VecPosition getMyRightGoalPost() {
        return ((Side == SIDE_LEFT)? worldObjects[GOALPOST_2_L].pos : worldObjects[GOALPOST_1_R].pos);
    }
    inline VecPosition getOppLeftGoalPost() {
        return ((Side == SIDE_LEFT)? worldObjects[GOALPOST_1_R].pos : worldObjects[GOALPOST_2_L].pos);
    }
    inline VecPosition getOppRightGoalPost() {
        return ((Side == SIDE_LEFT)? worldObjects[GOALPOST_2_R].pos : worldObjects[GOALPOST_1_L].pos );
    }

    inline double distanceToOppGoal(VecPosition &p) {

        VecPosition oppLeftGoalPost = getOppLeftGoalPost();
        VecPosition oppRightGoalPost = getOppRightGoalPost();

        if(p.getY() > oppLeftGoalPost.getY()) {
            return p.getDistanceTo(oppLeftGoalPost);
        }
        else if(p.getY() < oppRightGoalPost.getY()) {
            return p.getDistanceTo(oppRightGoalPost);
        }
        else {
            return fabs(oppLeftGoalPost.getX() - p.getX());
        }
    }

    inline double distanceToMyGoal(VecPosition &p) {

        VecPosition myLeftGoalPost = getMyLeftGoalPost();
        VecPosition myRightGoalPost = getMyRightGoalPost();

        if(p.getY() > myLeftGoalPost.getY()) {
            return p.getDistanceTo(myLeftGoalPost);
        }
        else if(p.getY() < myRightGoalPost.getY()) {
            return p.getDistanceTo(myRightGoalPost);
        }
        else {
            return fabs(myLeftGoalPost.getX() - p.getX());
        }
    }


    inline VecPosition getBall() const {
        return worldObjects[WO_BALL].pos;
    };
    inline void setBall(const VecPosition &ball) {
        worldObjects[WO_BALL].pos = ball;
    }

    inline VecPosition getBallWrtTorso() const {
        return g2l(getBall());
    }

    inline VecPosition getTeammate(const int &i) const {
        return worldObjects[i].pos;
    };
    inline void setTeammate(const int &i, const VecPosition &teammate) {
        worldObjects[i].pos = teammate;
    }

    inline VecPosition getOpponent(const int &i) const {
        return worldObjects[i].pos;
    };
    inline void setOpponent(const int &i, const VecPosition &opponent) {
        worldObjects[i].pos = opponent;
    }

    inline void setObjectPosition(const int &i, const VecPosition &pos) {
        worldObjects[i].pos = pos;
    }


    inline void setGlobalToLocal(const int &i, const int &j, const double &v) {
        this->globalToLocal.setCell(i, j, v);
    }

    inline void setLocalToGlobal(const int &i, const int &j, const double &v) {
        this->localToGlobal.setCell(i, j, v);
    }

//     // 定义一个函数，根据队员在世界模型中的索引i来返回队员的编号
// int WorldModel::getTeammateUNum(int i) {
//   // 如果索引i不合法，返回-1
//   if (i < 0 || i > NUM_AGENTS) {
//     return -1;
//   }
//   // 否则，返回索引i对应的队员编号
//   return TEAMMATE_NUM[i];
// }

    inline bool isLocalized() const {
        return fLocalized;
    }
    inline void setLocalized(bool fLocalized) {
        this->fLocalized = fLocalized;
    }

    inline bool isFallen() const {
        return fFallen;
    }
    inline void setFallen(bool fFallen) {
        this->fFallen = fFallen;
    }

    inline VecPosition g2l(const VecPosition &global) const {
        return globalToLocal.transform(global);
    }
    inline VecPosition l2g(const VecPosition &local) const {
        return localToGlobal.transform(local);
    }


    inline bool canTrustVision() {
        return fLocalized && getMyPosition().getDistanceTo(getMyLastPosition()) < .2;
    }

    bool getFallenTeammate(int index) const {
        return fallenTeammate[index];
    }
    void setFallenTeammate(int index, bool fFallen) {
        fallenTeammate[index] = fFallen;
    }
    bool getFallenOpponent(int index) const {
        return fallenOpponent[index];
    }
    void setFallenOpponent(int index, bool fFallen) {
        fallenOpponent[index] = fFallen;
    }

    string getOpponentTeamName() const {
        return opponentTeamName;
    }
    void setOpponentTeamName(string name) {
        opponentTeamName = name;
    }

    BallKF* getBallKalmanFilter() const {
        return ballKalmanFilter;
    }
    PlayerKF* getOpponentKalmanFilters() const {
        return opponentKalmanFilters;
    }

    double getTeammateLastHeardTime(int index) const {
        return teammateLastHeardTime[index];
    }
    void setTeammateLastHeardTime(int index, double time) {
        teammateLastHeardTime[index] = time;
    }

    void display();
};

#endif // WORLDMODEL_H

