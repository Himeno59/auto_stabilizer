#include "FootStepGenerator.h"
#include "MathUtil.h"

bool FootStepGenerator::setFootSteps(const GaitParam& gaitParam, const std::vector<StepNode>& footsteps,
                                     std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const{
  if(footsteps.size() <= 1) { // 何もしない
    o_footstepNodesList = gaitParam.footstepNodesList;
    return true;
  }

  std::vector<GaitParam::FootStepNodes> footstepNodesList;
  footstepNodesList.push_back(gaitParam.footstepNodesList[0]);

  // footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する
  if((GaitParam::isSupportPhaseEnd(footstepNodesList[0],RLEG) && GaitParam::isSupportPhaseEnd(footstepNodesList[0], LLEG)) && // footstepNodesList[0]の終了時の状態が両脚支持
     ((gaitParam.footstepNodesList.size() == 1) || ((GaitParam::isSupportPhaseStart(footstepNodesList[1], RLEG) && footsteps[1].l_r == RLEG) || (GaitParam::isSupportPhaseStart(footstepNodesList[1], LLEG) && footsteps[1].l_r == LLEG) || (!GaitParam::isSupportPhaseStart(footstepNodesList[1], RLEG) && footsteps[1].l_r == LLEG) || (GaitParam::isSupportPhaseStart(footstepNodesList[1],LLEG) && footsteps[1].l_r == RLEG)))
     ){
    this->extendDoubleSupportTime(footstepNodesList[0]);
  }

  // footstepNodesList[0]終了時にswing状態の足をfootstepNodesList[1]開始時にsupportにする必要がある場合は、footstepNodesList[0]の直後に両脚が横に並ぶ位置に一歩歩いてその足を下ろすnodeが挿入される.
  if((!GaitParam::isSupportPhaseEnd(footstepNodesList[0],RLEG) && footsteps[1].l_r == LLEG) || // RLEGを下ろす必要がある
     (!GaitParam::isSupportPhaseEnd(footstepNodesList[0],LLEG) && footsteps[1].l_r == RLEG)) { // LLEGを下ろす必要がある.
    GaitParam::FootStepNodes fs = this->calcDefaultNextStep(footstepNodesList[0], gaitParam.defaultTranslatePos);
    footstepNodesList.push_back(fs);
  }

  // footstepsの0番目の要素は、実際には歩かず、基準座標としてのみ使われる. footstepNodesList[0].dstCoordsのZ軸を鉛直に直した座標系と、footstepsの0番目の要素のZ軸を鉛直に直した座標系が一致するように座標変換する.
  cnoid::Position trans;
  {
    cnoid::Position genCoords = mathutil::orientCoordToAxis(footstepNodesList.back().dstCoords[footsteps[0].l_r], cnoid::Vector3::UnitZ());
    cnoid::Position refCoords = mathutil::orientCoordToAxis(footsteps[0].coords, cnoid::Vector3::UnitZ());
    trans = genCoords * refCoords.inverse();
  }

  for(int i=1;i<footsteps.size();i++){
    GaitParam::FootStepNodes fs;
    int swingLeg = footsteps[i].l_r;
    int supportLeg = swingLeg == RLEG ? LLEG: RLEG;
    fs.dstCoords[supportLeg] = footstepNodesList.back().dstCoords[supportLeg];
    fs.dstCoords[swingLeg] = trans * footsteps[i].coords;
    fs.supportTime[supportLeg] = std::numeric_limits<double>::max();
    fs.supportTime[swingLeg] = this->defaultDoubleSupportTime;
    if(footsteps[i].stepTime > this->defaultDoubleSupportTime){
      fs.remainTime = footsteps[i].stepTime;
    }else{
      fs.remainTime = this->defaultStepTime;
    }
    double stepHeight = std::max(0.0, footsteps[i].stepHeight);
    if(GaitParam::isSupportPhaseEnd(footstepNodesList.back(), swingLeg)){
      fs.stepHeight[swingLeg] = {stepHeight,stepHeight};
    }else{
      fs.stepHeight[swingLeg] = {0.0,stepHeight};
    }
    footstepNodesList.push_back(fs);
  }

  o_footstepNodesList = footstepNodesList;
  return true;
}


bool FootStepGenerator::goStop(const GaitParam& gaitParam,
            std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const {
  if(gaitParam.isStatic()){
    o_footstepNodesList = gaitParam.footstepNodesList;
    return true;
  }

  std::vector<GaitParam::FootStepNodes> footstepNodesList = gaitParam.footstepNodesList;

  // footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する
  if(footstepNodesList.size() == 1){
    if(GaitParam::isSupportPhaseEnd(footstepNodesList[0],RLEG) && GaitParam::isSupportPhaseEnd(footstepNodesList[0],LLEG)) {// footstepNodesList[0]の終了時の状態が両脚支持
      this->extendDoubleSupportTime(footstepNodesList[0]);
    }
  }

  // 両脚が横に並ぶ位置に2歩歩く.
  for(int i=0;i<2;i++){
    GaitParam::FootStepNodes fs = this->calcDefaultNextStep(footstepNodesList.back(), gaitParam.defaultTranslatePos);
    footstepNodesList.push_back(fs);
  }

  o_footstepNodesList = footstepNodesList;
  return true;

}

bool FootStepGenerator::calcFootSteps(const GaitParam& gaitParam, const double& dt,
                                      std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords) const{
  std::vector<GaitParam::FootStepNodes> footstepNodesList = gaitParam.footstepNodesList;

  // goVelocityModeなら、進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる
  if(this->isGoVelocityMode){
    // footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する
    if(footstepNodesList.size() == 1){
      if(GaitParam::isSupportPhaseEnd(footstepNodesList[0],RLEG) && GaitParam::isSupportPhaseEnd(footstepNodesList[0],LLEG)) { // footstepNodesList[0]の終了時の状態が両脚支持
        this->extendDoubleSupportTime(footstepNodesList[0]);
      }
    }

    cnoid::Position offset = cnoid::Position::Identity();
    offset.translation()[0] = this->cmdVel[0] * this->defaultStepTime;
    offset.translation()[1] = this->cmdVel[1] * this->defaultStepTime;
    offset.linear() = cnoid::Matrix3(Eigen::AngleAxisd(this->cmdVel[2] * this->defaultStepTime, cnoid::Vector3::UnitZ()));
    for(int i=footstepNodesList.size();i<=this->goVelocityStepNum; i++){
      footstepNodesList.push_back(this->calcDefaultNextStep(footstepNodesList.back(), gaitParam.defaultTranslatePos, offset));
    }
  }

  if(this->modifyFootSteps && this->isEmergencyStepMode){
    // TODO
  }

  // footstepNodesList[0].remainTimeが0で、後ろにfootstepNodesが追加されたなら消す
  std::vector<cnoid::Position> srcCoords = gaitParam.srcCoords;
  if(footstepNodesList[0].remainTime <= 0.0 && footstepNodesList.size() > 1){
    for(int i=0;i<NUM_LEGS;i++) srcCoords[i] = gaitParam.genCoords[i].value();
    footstepNodesList.erase(footstepNodesList.begin()); // vectorではなくlistにするべき?
  }

  if(this->modifyFootSteps){
    // TODO
  }

  o_footstepNodesList = footstepNodesList;
  o_srcCoords = srcCoords;
  return true;
}


GaitParam::FootStepNodes FootStepGenerator::calcDefaultNextStep(const GaitParam::FootStepNodes& footstepNodes, const std::vector<cnoid::Vector3>& defaultTranslatePos, const cnoid::Position& offset) const{
  GaitParam::FootStepNodes fs;
  fs.remainTime = this->defaultStepTime;
  if(!GaitParam::isSupportPhaseEnd(footstepNodes, RLEG)){ // ラストのstepで右脚が浮いた状態で終わっている
    fs.dstCoords[LLEG] = footstepNodes.dstCoords[LLEG]; // RLEGをswingする
    cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodes.dstCoords[LLEG], cnoid::Vector3::UnitZ());
    prevOrigin.translation() -= prevOrigin.linear() * defaultTranslatePos[LLEG];
    fs.dstCoords[RLEG] = prevOrigin * offset;
    fs.dstCoords[RLEG].translation() += fs.dstCoords[RLEG].linear() * defaultTranslatePos[RLEG];
    fs.supportTime[LLEG] = std::numeric_limits<double>::max();
    fs.supportTime[RLEG] = this->defaultDoubleSupportTime;
    fs.stepHeight[RLEG] = {0.0,this->defaultStepHeight}; // はじめを上げない
  }else if(!GaitParam::isSupportPhaseEnd(footstepNodes, LLEG)){ // ラストのstepで左脚が浮いた状態で終わっている
    fs.dstCoords[RLEG] = footstepNodes.dstCoords[RLEG]; // LLEGをswingする
    cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodes.dstCoords[RLEG], cnoid::Vector3::UnitZ());
    prevOrigin.translation() -= prevOrigin.linear() * defaultTranslatePos[RLEG];
    fs.dstCoords[LLEG] = prevOrigin * offset;
    fs.dstCoords[LLEG].translation() += fs.dstCoords[LLEG].linear() * defaultTranslatePos[LLEG];
    fs.supportTime[RLEG] = std::numeric_limits<double>::max();
    fs.supportTime[LLEG] = this->defaultDoubleSupportTime;
    fs.stepHeight[LLEG] = {0.0,this->defaultStepHeight}; // はじめを上げない
  }else if((footstepNodes.remainTime <= footstepNodes.supportTime[RLEG]) && (footstepNodes.remainTime <= footstepNodes.supportTime[LLEG])){ // ラストのstepで両脚ともswingしていない
    fs.dstCoords[LLEG] = footstepNodes.dstCoords[LLEG]; // どっちをswingしてもいい. RLEGをswingする
    cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodes.dstCoords[LLEG], cnoid::Vector3::UnitZ());
    prevOrigin.translation() -= prevOrigin.linear() * defaultTranslatePos[LLEG];
    fs.dstCoords[RLEG] = prevOrigin * offset;
    fs.dstCoords[RLEG].translation() += fs.dstCoords[RLEG].linear() * defaultTranslatePos[RLEG];
    fs.supportTime[LLEG] = std::numeric_limits<double>::max();
    fs.supportTime[RLEG] = this->defaultDoubleSupportTime;
    fs.stepHeight[RLEG] = {this->defaultStepHeight,this->defaultStepHeight};
  }else if(footstepNodes.supportTime[LLEG] > footstepNodes.supportTime[RLEG]){ //ラストのstepで最後に右足をtouch groundした
    fs.dstCoords[RLEG] = footstepNodes.dstCoords[RLEG]; // LLEGをswingする
    cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodes.dstCoords[RLEG], cnoid::Vector3::UnitZ());
    prevOrigin.translation() -= prevOrigin.linear() * defaultTranslatePos[RLEG];
    fs.dstCoords[LLEG] = prevOrigin * offset;
    fs.dstCoords[LLEG].translation() += fs.dstCoords[LLEG].linear() * defaultTranslatePos[LLEG];
    fs.supportTime[RLEG] = std::numeric_limits<double>::max();
    fs.supportTime[LLEG] = this->defaultDoubleSupportTime;
    fs.stepHeight[LLEG] = {this->defaultStepHeight,this->defaultStepHeight};
  }else{  //ラストのstepで最後に左足をtouch groundした. または両足同時.
    fs.dstCoords[LLEG] = footstepNodes.dstCoords[LLEG]; // RLEGをswingする
    cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodes.dstCoords[LLEG], cnoid::Vector3::UnitZ());
    prevOrigin.translation() -= prevOrigin.linear() * defaultTranslatePos[LLEG];
    fs.dstCoords[RLEG] = prevOrigin * offset;
    fs.dstCoords[RLEG].translation() += fs.dstCoords[RLEG].linear() * defaultTranslatePos[RLEG];
    fs.supportTime[LLEG] = std::numeric_limits<double>::max();
    fs.supportTime[RLEG] = this->defaultDoubleSupportTime;
    fs.stepHeight[RLEG] = {this->defaultStepHeight,this->defaultStepHeight};
  }

  // limit stride. TODO
  return fs;
}

void FootStepGenerator::extendDoubleSupportTime(GaitParam::FootStepNodes& footstepNodes) const{
  if((GaitParam::isSupportPhaseEnd(footstepNodes,RLEG) && GaitParam::isSupportPhaseEnd(footstepNodes,LLEG)) && // footstepNodesの終了時の状態が両脚支持
     std::min({footstepNodes.remainTime, footstepNodes.supportTime[RLEG], footstepNodes.supportTime[LLEG]}) < this->defaultDoubleSupportTime
     ){
    double extendTime =this->defaultDoubleSupportTime - std::min({footstepNodes.remainTime, footstepNodes.supportTime[RLEG], footstepNodes.supportTime[LLEG]});
    footstepNodes.remainTime += extendTime;
    footstepNodes.supportTime[RLEG] += extendTime;
    footstepNodes.supportTime[LLEG] += extendTime;
  }
}