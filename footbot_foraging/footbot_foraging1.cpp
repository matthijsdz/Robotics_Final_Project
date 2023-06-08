/* Include the controller definition */
#include "footbot_foraging1.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

#include <typeinfo>

#include <cmath>

#include <vector>

/****************************************/
/* Class with data about food           */
/****************************************/

CFootBotForaging::SFoodData::SFoodData() :
   HasFoodItem(false),
   FoodItemIdx(0),
   TotalFoodItems(0) {}

void CFootBotForaging::SFoodData::Reset() {
   HasFoodItem = false;
   FoodItemIdx = 0;
   TotalFoodItems = 0;
}

/****************************************/
/* Class for footbot diffusion          */
/****************************************/

CFootBotForaging::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotForaging::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/* Class for truning the robot          */
/****************************************/

void CFootBotForaging::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/* Class for robot state                */
/****************************************/

CFootBotForaging::SStateData::SStateData() :
   ProbRange(0.0f, 1.0f) {}

void CFootBotForaging::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "initial_rest_to_explore_prob", InitialRestToExploreProb);
      GetNodeAttribute(t_node, "initial_explore_to_rest_prob", InitialExploreToRestProb);
      GetNodeAttribute(t_node, "food_rule_explore_to_rest_delta_prob", FoodRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "food_rule_rest_to_explore_delta_prob", FoodRuleRestToExploreDeltaProb);
      GetNodeAttribute(t_node, "collision_rule_explore_to_rest_delta_prob", CollisionRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "minimum_resting_time", MinimumRestingTime);
      GetNodeAttribute(t_node, "minimum_unsuccessful_explore_time", MinimumUnsuccessfulExploreTime);
      GetNodeAttribute(t_node, "minimum_search_for_place_in_nest_time", MinimumSearchForPlaceInNestTime);

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}

void CFootBotForaging::SStateData::Reset() {
   State = STATE_RESTING;
   InNest = true;
   RestToExploreProb = InitialRestToExploreProb;
   ExploreToRestProb = InitialExploreToRestProb;
   TimeExploringUnsuccessfully = 0;
   TimeRested = MinimumRestingTime;
   TimeSearchingForPlaceInNest = 0;
}

/****************************************/
/****************************************/

CFootBotForaging::CFootBotForaging() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcProximity(NULL),
   m_pcLight(NULL),
   m_pcGround(NULL),
   m_pcRNG(NULL) {}

/****************************************/
/****************************************/

void CFootBotForaging::Init(TConfigurationNode& t_node) {
   try {
      /*
       * Initialize sensors/actuators
       */
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
      m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
      m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
      m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
      m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );
      m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"          );
      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot foraging controller for robot \"" << GetId() << "\"", ex);
   }
   /*
    * Initialize other stuff
    */
   /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
   m_pcRNG = CRandom::CreateRNG("argos");
   Reset();
}

/****************************************/
/****************************************/

void CFootBotForaging::ControlStep() {
   switch(m_sStateData.State) {
      case SStateData::STATE_RESTING: {
         Rest();
         break;
      }
      case SStateData::STATE_TO_TARGET: {
         ToTarget();
         break;
      }
      case SStateData::STATE_EXPLORING: {
         Explore();
         break;
      }
      case SStateData::STATE_RETURN_TO_NEST: {
         ReturnToNest();
         break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
   }
}

/****************************************/
/* changed                              */
/****************************************/

void CFootBotForaging::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Reset food data */
   m_sFoodData.Reset();
   /* Set LED color */
   m_pcLEDs->SetAllColors(CColor::RED);
   /* Clear up the last exploration result */
   m_pcRABA->ClearData();
}

/****************************************/
/****************************************/

void CFootBotForaging::UpdateState() {
   /* Reset state flags */
   m_sStateData.InNest = false;
   /* Read stuff from the ground sensor */
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
   if(tGroundReads[2].Value > 0.25f &&
      tGroundReads[2].Value < 0.75f &&
      tGroundReads[3].Value > 0.25f &&
      tGroundReads[3].Value < 0.75f) {
      m_sStateData.InNest = true;
   }
}

/****************************************/
/****************************************/

CVector2 CFootBotForaging::CalculateVectorToLight() {
   /* Get readings from light sensor */
   const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tLightReads.size(); ++i) {
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   }
   /* If the light was perceived, return the vector */
   if(cAccumulator.Length() > 0.0f) {
      return CVector2(1.0f, cAccumulator.Angle());
   }
   /* Otherwise, return zero */
   else {
      return CVector2();
   }
}

/*-----------------------------------------------------------------------------*/
/*                  Function to calculate vector to target                     */
/*-----------------------------------------------------------------------------*/

CVector2 CFootBotForaging::CalculateTargetVector() {

   CVector2 target = FoodPos;
   CVector3 GetPosition = m_pcPosSens->GetReading().Position;
   CVector2 position = CVector2(GetPosition.GetX(),GetPosition.GetY());
   CQuaternion o = m_pcPosSens->GetReading().Orientation; 
   CRadians cZAngle, cYAngle, cXAngle;
   o.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector2 toTarget(target - position);
   toTarget.Rotate(-cZAngle);
   
   return toTarget;
}
/*-----------------------------------------------------------------------------*/

/****************************************/
/****************************************/

CVector2 CFootBotForaging::DiffusionVector(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CFootBotForaging::Rest() {
   /* If we have stayed here enough, probabilistically switch to
    * 'exploring' */
   if(m_sStateData.TimeRested > m_sStateData.MinimumRestingTime) {
      m_pcLEDs->SetAllColors(CColor::GREEN);
      m_sStateData.State = SStateData::STATE_EXPLORING;
      m_sStateData.TimeRested = 0;
   }
   else {
      ++m_sStateData.TimeRested;
      if(m_sStateData.TimeRested == 1) {
         m_pcRABA->ClearData();
      }
   }
}

/****************************************/

void CFootBotForaging::ToTarget() {
   m_pcLEDs->SetAllColors(CColor::YELLOW);
   float min_distance = 0.5;
   float distance = 0;
   
   //keep exploration time, so exploration after reaching target does not take to long
   ++m_sStateData.TimeExploringUnsuccessfully;
   UpdateState();
   // calculate distance to target
   CVector2 target = FoodPos;
   CVector3 GetPosition = m_pcPosSens->GetReading().Position;
   CVector2 position = CVector2(GetPosition.GetX(),GetPosition.GetY());
   distance = sqrt(pow((position.GetX()-target.GetX()),2) + pow((position.GetY()-target.GetY()),2));
 
   /*-----------------------------------------------------------------------------*/
   /*   in case robot encounter food on the way: return to nest at once (old code)*/
   /*-----------------------------------------------------------------------------*/

   if(m_sFoodData.HasFoodItem) {

      m_sStateData.ExploreToRestProb -= m_sStateData.FoodRuleExploreToRestDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
      m_sStateData.RestToExploreProb += m_sStateData.FoodRuleRestToExploreDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);


      m_sStateData.TimeExploringUnsuccessfully = 0;
      m_sStateData.TimeSearchingForPlaceInNest = 0;
      m_pcLEDs->SetAllColors(CColor::BLUE);
      m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
   /*-----------------------------------------------------------------------------*/
      HasTarget = false; 
   }
   /*-----------------------------------------------------------------------------*/
   /*     if robot is close enough to target state, it start to explore (new code)*/
   /*-----------------------------------------------------------------------------*/
   else if (distance < min_distance) {
      m_pcLEDs->SetAllColors(CColor::GREEN);

      bool bCollision;
      CVector2 cDiffusion = DiffusionVector(bCollision);                     // get diffusion vector as well
   
      
      CRange<Real> range = -1.0;
      float random_x = 0.5 + m_pcRNG->Uniform(range);
      float random_y = 0.5 + m_pcRNG->Uniform(range);
      LOGERR << "vector = " << random_x << " " << random_y << std::endl;
      SetWheelSpeedsFromVector(
          m_sWheelTurningParams.MaxSpeed * 0.05f * cDiffusion +               // diffusion
          m_sWheelTurningParams.MaxSpeed * 2.0f * CVector2(random_x,random_y) -   // main component: vector towards target
          m_sWheelTurningParams.MaxSpeed * 0.10f * CalculateVectorToLight()); // -1 * vector-to-light:  move away from the lights 

      m_sStateData.State = SStateData::STATE_EXPLORING; 
   }
   /*-----------------------------------------------------------------------------*/
   /*     else: robot moves towards target using the target vector (new code)     */
   /*-----------------------------------------------------------------------------*/   
   else { 
      CVector2 cTarget = CalculateTargetVector();                            // get vector to target 

      bool bCollision;
      CVector2 cDiffusion = DiffusionVector(bCollision);                     // get diffusion vector as well
      /*--------------------------------------------------------------------------*/
      /*  Apply the collision rule, if a collision avoidance happened (Old code)  */
      /*--------------------------------------------------------------------------*/
      if(bCollision) {
         m_sStateData.ExploreToRestProb += m_sStateData.CollisionRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
         m_sStateData.RestToExploreProb -= m_sStateData.CollisionRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      }
      /*--------------------------------------------------------------------------*/
      /*  else: robot moves towards target using the target vector  (new code)    */
      /*--------------------------------------------------------------------------*/    
      SetWheelSpeedsFromVector(
          m_sWheelTurningParams.MaxSpeed * 0.05f * cDiffusion +               // diffusion
          m_sWheelTurningParams.MaxSpeed * 2.0 * cTarget -                    // main component: vector towards target
          m_sWheelTurningParams.MaxSpeed * 0.10f * CalculateVectorToLight()); // -1 * vector-to-light:  move away from the lights 
      /*--------------------------------------------------------------------------*/    
   }
}

/****************************************/

void CFootBotForaging::Explore() { 


   bool bReturnToNest(false);

   if(m_sFoodData.HasFoodItem) {
      m_sStateData.ExploreToRestProb -= m_sStateData.FoodRuleExploreToRestDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
      m_sStateData.RestToExploreProb += m_sStateData.FoodRuleRestToExploreDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      bReturnToNest = true;
   }
   else if(m_sStateData.TimeExploringUnsuccessfully > m_sStateData.MinimumUnsuccessfulExploreTime) {
      if (m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.ExploreToRestProb) {
         bReturnToNest = true;
      }
      else {
         m_sStateData.ExploreToRestProb += m_sStateData.FoodRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
         m_sStateData.RestToExploreProb -= m_sStateData.FoodRuleRestToExploreDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      }
   }
   if(bReturnToNest) {
      m_sStateData.TimeExploringUnsuccessfully = 0;
      m_sStateData.TimeSearchingForPlaceInNest = 0;
      m_pcLEDs->SetAllColors(CColor::BLUE);
      /*------------------------------------------------------------------------------*/
      /* if food found, read position sensor to get cuttent coordinates and store them*/
      /*------------------------------------------------------------------------------*/
      if(m_sFoodData.HasFoodItem) {
         CVector3 Sdata = m_pcPosSens->GetReading().Position;
         FoodPos = CVector2(Sdata.GetX(),Sdata.GetY());
      }
      m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
      HasTarget = false; 
      /*------------------------------------------------------------------------------*/
   }
   else {
      /* No, perform the actual exploration */
      ++m_sStateData.TimeExploringUnsuccessfully;
      UpdateState();


      /*###################################################*/
      /* READ RABS and if there are targets, chose one     */
      /*  and switch to state TO_TARGET                    */
      /*###################################################*/
      CVector2 Coordinate;
      union Uc {
         float f;
      	 unsigned char c[4];
      };
      if (HasTarget == false && m_sStateData.TimeExploringUnsuccessfully > 10){
      //if (HasTarget == false){
         // read sensor
         const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

         // find all sensor packets that contain info about target. 
         bool FoundTarget = false;

         for(size_t i = 0; i < tPackets.size(); ++i) {

      	    // find tpackets that contain information
      	    if (tPackets[i].Data[0] == 1){ // contains location

      	       // convert data [1:9] to x and y coordinate
      	       Uc x,y;
      	       for (int j=0; j < 4; j++){
      		   x.c[j] = tPackets[i].Data[j+1];
      		   y.c[j] = tPackets[i].Data[j+5];

      		}
         	// [x,y] --> vector
         	Coordinate = CVector2(x.f,y.f);
         	FoundTarget = true;
         	break;

            }
         }
      // if any packets are found chose one vector from array and switch to STATE::TO_TARGET 
         if (FoundTarget == true){
        
      	   FoodPos = Coordinate;
      	   // robot cannot take another target till it has returned to the nest again
      	   HasTarget = true;

      	   m_pcLEDs->SetAllColors(CColor::YELLOW);
     	   m_sStateData.State = SStateData::STATE_TO_TARGET; 
     	}	
     }      

     


      

      

      /* Get the diffusion vector to perform obstacle avoidance */
      bool bCollision;
      CVector2 cDiffusion = DiffusionVector(bCollision);
      /* Apply the collision rule, if a collision avoidance happened */
      if(bCollision) {
         /* Collision avoidance happened, increase ExploreToRestProb and
          * decrease RestToExploreProb */
         m_sStateData.ExploreToRestProb += m_sStateData.CollisionRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
         m_sStateData.RestToExploreProb -= m_sStateData.CollisionRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      }
      /*
       * If we are in the nest, we combine antiphototaxis with obstacle
       * avoidance
       * Outside the nest, we just use the diffusion vector
       */
      if(m_sStateData.InNest) {
         /*
          * The vector returned by CalculateVectorToLight() points to
          * the light. Thus, the minus sign is because we want to go away
          * from the light.
          */
         SetWheelSpeedsFromVector(
            m_sWheelTurningParams.MaxSpeed * cDiffusion -
            m_sWheelTurningParams.MaxSpeed * 0.25f * CalculateVectorToLight());
      }
      else {
         /* Use the diffusion vector only */
         SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
      }
   }
}

/*---------------------------------------------------------------------------*/
/*                               Return to nest                              */
/*---------------------------------------------------------------------------*/

void CFootBotForaging::ReturnToNest() {

   UpdateState();

   if(m_sStateData.InNest) {                                                                       //If robot is in the nest 
   
      if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime){ // - enough space in nest
         m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
         m_pcLEDs->SetAllColors(CColor::RED);
         m_sStateData.State = SStateData::STATE_RESTING;
         m_sStateData.TimeSearchingForPlaceInNest = 0;
         return;
      }
      
      else {                                                                                      //  - not enough space
         ++m_sStateData.TimeSearchingForPlaceInNest;
      }
      
   }
   
   else {                                                                                        //Else (robot not in nest)
      m_sStateData.TimeSearchingForPlaceInNest = 0;
      /*----------------------------------------------------------*/
      /* Sending foor coordinates over Range And Bearing Actuator */
      /*----------------------------------------------------------*/
      CByteArray cBuf(10);
      
      union Uc {
      	float f;
      	unsigned char c[4];
      };
       
      Uc x,y;
      x.f = FoodPos.GetX();
      y.f = FoodPos.GetY();
      
      cBuf[0] = 1;
      cBuf[9] = 1;
      for (int i=0; i < 4; i++){
      	cBuf[i+1] = x.c[i];
      }
      for (int i=0; i < 4; i++){
        cBuf[i+5] = y.c[i];
      }
      m_pcRABA->SetData(cBuf);
      /*----------------------------------------------------------*/
   }
   
   bool bCollision;
   SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) + m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
}


/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotForaging, "footbot_foraging_controller")
