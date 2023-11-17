

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/CubicSplineFactory.hpp>


#include <rw/invkin.hpp>
#include <rw/rw.hpp>
#include <math.h> 
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>
#include <list>
#include <sstream>

#include <fstream>

#include <chrono>

using namespace rw::loaders;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rw::models;
using namespace rw::kinematics;




std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp,
                                    rw::models::SerialDevice::Ptr robot,
                                    rw::models::WorkCell::Ptr wc, State& state)
{
    // Get, make and print name of frames
    const std::string robotName     = robot->getName ();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp  = robotName + "." + "TCP";

    // Find frames and check for existence
    Frame::Ptr goal_f      = wc->findFrame (nameGoal);
    Frame::Ptr tcp_f       = wc->findFrame (nameTcp);
    Frame::Ptr robotBase_f = wc->findFrame (nameRobotBase);
    Frame::Ptr robotTcp_f  = wc->findFrame (nameRobotTcp);
    if (goal_f.isNull () || tcp_f.isNull () || robotBase_f.isNull () || robotTcp_f.isNull ()) {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (goal_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (tcp_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameRobotBase
                  << "\": " << (robotBase_f.isNull () ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp
                  << "\": " << (robotTcp_f.isNull () ? "NO!" : "YES!") << std::endl;
    }


    
    
    // Make "helper" transformations

    //Transform3D<> aboveGoal=baseTGoal*rotate*translate;

    std::list<Transform3D<>> goals;
    //goals.push_back(baseTGoal*rotate*translate);

    // get grasp frame in robot tool frame


    Transform3D<> baseTGoal    = Kinematics::frameTframe (robotBase_f, goal_f, state);
    Transform3D<> tcpTRobotTcp = Kinematics::frameTframe (tcp_f, robotTcp_f, state);

    // get grasp frame in robot tool frame
    Transform3D<> targetAt = baseTGoal * tcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler =
        rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (robot, state));

    return closedFormSovler->solve (targetAt, state);

    

    

}

double adjustAngleForShortestPath(double fromAngle, double toAngle) {
    double difference = toAngle - fromAngle;

    // Adjusting if the difference is more than 180 degrees
    if (difference > M_PI) {
        toAngle -= 2 * M_PI;
    } else if (difference < -M_PI) {
        toAngle += 2 * M_PI;
    }

    return toAngle;
}


//TimedStatePath linInterp (Device::Ptr device, State state, MovableFrame::Ptr cylinderFrame, Frame* tool_frame, 
//        std::vector< Q > points, double duration, std::ofstream& file, rw::proximity::CollisionDetector& detector)
//{
    //TimedStatePath res;



    /*
    //LinearInterpolator< Q > interp (from, to, duration);
    cylinderFrame->attachTo(tool_frame,state);
    cylinderFrame->setTransform(Transform3D<> (Vector3D<>(0.0, 0.0, 0.0),RPY<> (0, 0, 0)), state);


    Path< Timed< Q > >	path;
    path.push_back(Timed<Q>(0,points[0]));
    double counter=0;
    for(int n=0;n<points.size()-1;n++){
        counter+=1.0;


        path.push_back(Timed<Q>(counter,points[n+1]));

    }*/

    //Transform3D<> T1(Vector3D<>(0,0,0), EAA<>(0,0,0));
    //Transform3D<> T2(Vector3D<>(1,1,0), EAA<>(1,1,0));
    //Transform3D<> T3(Vector3D<>(2,0,0), EAA<>(2,2,0));
 //
    //LinearInterpolator<Transform3D<> >::Ptr cartInt1 = ownedPtr(new LinearInterpolator<Transform3D<> >(T1, T2, 1));
    //LinearInterpolator<Transform3D<> >::Ptr cartInt2 = ownedPtr(new LinearInterpolator<Transform3D<> >(T2, T3, 1));
    //ParabolicBlend<Transform3D<> >::Ptr blend1 = ownedPtr(new ParabolicBlend<Transform3D<> >(cartInt1, cartInt2, 0.25));
    //InterpolatorTrajectory<Transform3D<> > trajectory;
    //trajectory.add(cartInt1);
    //trajectory.add(blend1, cartInt2);
    //std::ofstream out("test.dat");


    /*
    double dt=0.05;
    InterpolatorTrajectory< Q >::Ptr interp = rw::trajectory::CubicSplineFactory::makeNaturalSpline(path);

    for (double t = 0; t<=interp->duration(); t += dt) {
    

        device->setQ(interp->x(t),state);
        

        Transform3D<> transform= tool_frame->wTf(state);

        file<<transform.P()[0]<<" " <<transform.P()[1] << " " << transform.P()[2] << " ";

        res.push_back(TimedState(t,state));
    }
    */

double JointSpaceDistance(const Q& from, const Q& to) {
    double distance = 0.0;
    for (size_t i = 0; i < from.size(); ++i) {
        distance += std::pow(to[i] - from[i], 2);
    }
    return std::sqrt(distance);
}

TimedStatePath linInterp (Device::Ptr device, State state, MovableFrame::Ptr cylinderFrame, Frame* tool_frame, Frame* world_frame, 
        std::vector< Q > points, double duration, std::ofstream& file, rw::proximity::CollisionDetector& detector, bool doGribFromAbove)
{

    TimedStatePath res;
    std::stringstream strData("");

    //LinearInterpolator< Q > interp (from, to, duration);
    //cylinderFrame->attachTo(tool_frame,state);
    //cylinderFrame->setTransform(Transform3D<> (Vector3D<>(0.0, 0.0, 0.0),RPY<> (0, 0, 0)), state);

    for(int n=0;n<points.size()-1;n++){

        


        for (int j = 0; j < points[n].size(); ++j) {
            if(j==2){
                continue;
            }
            points[n+1][j] = adjustAngleForShortestPath(points[n][j],points[n+1][j]); // Ajust for shortest path being the other way around circle

        }
        if(n==2 ){
            cylinderFrame->attachTo(tool_frame,state);
            if(!doGribFromAbove){
                cylinderFrame->setTransform(Transform3D<> (Vector3D<>(0, 0, 0),RPY<> (0, 0, 0)), state);
            }else{

                cylinderFrame->setTransform(Transform3D<> (Vector3D<>(0, 0, 0.05),RPY<> (0, 0, -1.57)), state);
            }
        }
        if(n==4){
            cylinderFrame->attachTo(world_frame,state);
            cylinderFrame->setTransform(Transform3D<> (Vector3D<>(0.3,-0.475,0.116),RPY<> (0, 0, 1.57)), state);
        }

        double distance = JointSpaceDistance(points[n], points[n+1]);
        double v=1.5;

        LinearInterpolator< Q > interp (points[n], points[n+1], distance/v);
        for (double i = 0; i < distance/v; i += 0.03333) {
            
            
            //Attach and detach Obejct End

            //double x = 0;
            //if(i>0.5*distance/v) x=distance/v;
            device->setQ (interp.x (i), state);
            Q currentQ = interp.x (i);
            //cylinderFrame->moveTo(tool_frame->getTransform(state),state);
            Transform3D<> transform= tool_frame->wTf(state);

            for (size_t i = 0; i < currentQ.size(); ++i) {
                //strData << currentQ[i] << " ";
            }

            for (size_t i = 0; i < 3; ++i) {
                //strData << currentQ[i] << " ";
                strData << transform.P()[i] << " ";
            }
            for (size_t i = 0; i < 3; ++i) {
                //strData << currentQ[i] << " ";
                RPY<> rot = RPY(transform.R());
                strData << rot[i] << " ";
            }
            strData << std::endl;

            //if(detector.inCollision (state)){
            //    return TimedStatePath();
            //}

            res.push_back (TimedState (i, state));
        }
    }
    
    //strData<<std::endl;
    file<<strData.rdbuf();

    return res;
}


TimedStatePath quadInterp (Device::Ptr device, State state, MovableFrame::Ptr cylinderFrame, Frame* tool_frame, Frame* world_frame, 
        std::vector< Q > points, double duration, std::ofstream& file, rw::proximity::CollisionDetector& detector, bool doGribFromAbove)
{

    TimedStatePath res;
    std::stringstream strData("");

    //LinearInterpolator< Q > interp (from, to, duration);
    //cylinderFrame->attachTo(tool_frame,state);
    //cylinderFrame->setTransform(Transform3D<> (Vector3D<>(0.0, 0.0, 0.0),RPY<> (0, 0, 0)), state);

    for(int n=0;n<points.size()-1;n++){
        for (int j = 0; j < points[n].size(); ++j) {
            if(j==2){
                continue;
            }
            points[n+1][j] = adjustAngleForShortestPath(points[n][j],points[n+1][j]); // Ajust for shortest path being the other way around circle

        }

        if(n==2 ){
            cylinderFrame->attachTo(tool_frame,state);
            if(!doGribFromAbove){
                cylinderFrame->setTransform(Transform3D<> (Vector3D<>(0, 0, 0),RPY<> (0, 0, 0)), state);
            }else{

                cylinderFrame->setTransform(Transform3D<> (Vector3D<>(0, 0, 0.05),RPY<> (0, 0, -1.57)), state);
            }
        }
        if(n==4){
            cylinderFrame->attachTo(world_frame,state);
            cylinderFrame->setTransform(Transform3D<> (Vector3D<>(0.3,-0.475,0.116),RPY<> (0, 0, 1.57)), state);
        }


        double distance = JointSpaceDistance(points[n], points[n+1]);
        double dynamicDuration = distance*2;

        double blendTime = std::min(dynamicDuration / 2, 0.5);
        double linearDuration = dynamicDuration - (2 * blendTime);

        //double V=1.0;
        //V=std::min(V,(2*(distance)/linearDuration))
        //double a = V/blendTime



        double t_acc = 0.2;
        double q = 0.0;

        double t1, t2, s1, s2, linearDist,tLinear,tTotal;

        double v=3;
        double a=5;

        double tf=1;

        t1 = v/a; //Time taken to reach max speed

        s1 = 0.5*a*t1*t1; //Distance at max speed

        s1 = std::min(distance/2,s1); // can maximum reach half total distance

        t1 = std::sqrt(2*s1/a); //checks what time the max speed is reached
        v=a*t1; // And what the maximum speed reaches is


        linearDist = distance - 2*s1;
        s2 = s1+linearDist;

        tLinear=linearDist/v;


        t2 = t1+tLinear;

        tTotal=t1*2+tLinear;

        for (double t = 0; t <= tTotal; t += 0.03333) {
            Q currentQ;


            // if (t < blendTime) {
            //     // Parabolic acceleration phase
                
            //     double q = ((t * t)/(2*blendTime));

            //     currentQ = points[n] + q * (points[n+1] - points[n]);
            // } else if (t < (blendTime + linearDuration)) {
            //     // Linear phase
            //     double alpha = (t - blendTime) / linearDuration;


            //     double qB = ((blendTime * blendTime)/(2*blendTime));
            //     double q=qB + ((t-blendTime)/linearDuration);
            //     currentQ = points[n] + q * (points[n+1] - points[n]);
        
            // } else {
            //     // Parabolic deceleration phase
            //     double alpha = dynamicDuration-t; // Normalized time for blend phase
            //     double q = ((alpha * alpha)/(2*blendTime));
            //     currentQ = points[n+1] - q * (points[n+1] - points[n]);
            // }
            // Rest of your existing code for setting the state, handling attachment, etc.


            
            //double a = 10;

            // if (t < t_acc) {
            //     // Acceleration phase
            //     q = (t / t_acc) * (t / t_acc);
            // } else if (t < 1.0 - t_acc) {
            //     // Linear phase
            //     q = 2 * t - t_acc;
            // } else {
            //     // Deceleration phase
            //     double t_rel = (t - (1.0 - t_acc)) / t_acc;
            //     q = 1.0 - (1.0 - t_rel) * (1.0 - t_rel);
            // }


            // if(t<t_acc){
            //     q=0+0.5*a*(0+t)*(0+t);
            // }else if(t<1-t_acc){
            //     q=0+a*t_acc*(t-(t_acc/2));
            // }else{
            //     q=1-0.5*a*(1-t)*(1-t);
            // }

            

            // if (t <= t1) {
            //     double a = 0.1 / (t1 * t1);
            //     q = a * t * t;
            // }
            // else if (t <= t2) {
            //     double m = (0.9 - 0.1) / (t2 - t1);
            //     double c = 0.1 - m * t1;
            //     q = m * t + c;
            // }
            // else {
            //     double a = 0.1 / (t1 * t1);
            //     double tf = 1-t;
            //     q = q = 1-(a * tf * tf);
            // }

            

            if (t <= t1) {
                double a = s1 / (t1 * t1);
                q = a * t * t;
            }
            else if (t <= t2) {
                double m = (s2 - s1) / (t2 - t1);
                double c = s1 - m * t1;
                q = m * t + c;
            }
            else {
                double a = s1 / (t1 * t1);
                double tf = tTotal-t;
                q = s2+s1-(a * tf * tf);
            }
            q=q/distance;
              
            std::cout<<"t: "<<t<<" q: "<<q<<std::endl;



            currentQ = points[n] + q * (points[n+1] - points[n]);

            device->setQ(currentQ, state);
            Transform3D<> transform = tool_frame->wTf(state);
            //strData << transform.P()[0] << transform.P()[1] << " " << transform.P()[2] << " ";
            for (size_t i = 0; i < currentQ.size(); ++i) {
                //strData << currentQ[i] << " ";
                
            }
            for (size_t i = 0; i < 3; ++i) {
                //strData << currentQ[i] << " ";
                strData << transform.P()[i] << " ";
            }
            for (size_t i = 0; i < 3; ++i) {
                //strData << currentQ[i] << " ";
                RPY<> rot = RPY(transform.R());
                strData << rot[i] << " ";
            }
            strData << std::endl;

            res.push_back(TimedState(t, state));
        }
        //return res;
    }
    
    //strData<<std::endl;
    file<<strData.rdbuf();

    return res;
}


Q getConfigFromPos(WorkCell::Ptr wc, State& state, rw::proximity::CollisionDetector& detector, 
MovableFrame::Ptr cylinderFrame, SerialDevice::Ptr robotUR5, Vector3D<> pos, double rot,Q previousConfig, bool doGribFromAbove=false){

    std::cout<<"Finding Solutions"<<std::endl;
    Transform3D<> cylPos;
    if(doGribFromAbove){
        pos[2]=pos[2]+0.05;
        cylPos=Transform3D<> (pos,RPY<> (rot, 0, 3.142));
    }else{

        
        cylPos=Transform3D<> (pos,RPY<> (rot, 0, 1.6));
    }
    cylinderFrame->moveTo (cylPos, state);

    std::vector< Q > stateSolutions = getConfigurations ("Bottle", "GraspTCP", robotUR5, wc, state);

    //cylPos=Transform3D<> (pos,RPY<> (rot+1.6, 0, 1.6));
    //cylinderFrame->moveTo (cylPos, state);

    //std::vector< Q > stateSolutions2 = getConfigurations ("Bottle", "GraspTCP", robotUR5, wc, state);
    //stateSolutions.insert(stateSolutions.end(),stateSolutions2.begin(),stateSolutions2.end());
    
    //cylPos=Transform3D<> (pos,RPY<> (rot-1.6, 0, 1.6));
    //cylinderFrame->moveTo (cylPos, state);

    //stateSolutions2 = getConfigurations ("Bottle", "GraspTCP", robotUR5, wc, state);
    //stateSolutions.insert(stateSolutions.end(),stateSolutions2.begin(),stateSolutions2.end());
    //std::cout<<stateSolutions.size()<<std::endl;
    /*
    cylPos=Transform3D<> (pos,RPY<> (rot-1.6, 0, 1.6));
    cylinderFrame->moveTo (cylPos, state);

    stateSolutions2 = getConfigurations ("Bottle", "GraspTCP", robotUR5, wc, state);
    stateSolutions.insert(stateSolutions.end(),stateSolutions2.begin(),stateSolutions2.end());

    cylPos=Transform3D<> (pos,RPY<> (rot-1.6, 0, 1.6));
    cylinderFrame->moveTo (cylPos, state);

    stateSolutions2 = getConfigurations ("Bottle", "GraspTCP", robotUR5, wc, state);
    stateSolutions.insert(stateSolutions.end(),stateSolutions2.begin(),stateSolutions2.end());
    */

    
    //std::cout<<pos<<std::endl;


    float bestDist=999;
    int bestSolution=0;
    rw::proximity::CollisionDetector::QueryResult* collision = new rw::proximity::CollisionDetector::QueryResult();
    for (unsigned int i = 0; i < stateSolutions.size (); i++) {
            // set the robot in that configuration and check if it is in collision
        robotUR5->setQ (stateSolutions[i], state);
        detector.inCollision (state, collision, true);
        //std::cout<<detector.inCollision (state, collision, true)<<std::endl;
        if (detector.inCollision (state)) {
            std::cout<<collision->collidingFrames.begin()->first->getName()<< " And " << collision->collidingFrames.begin()->second->getName()<<" Collided"<<std::endl;
        }else{
            std::cout<<"No Collision"<<std::endl;
        }
        if (!detector.inCollision (state)) {
            //bestSolution=i;
            
            

            double sum=0;
            for(int n=0;n<6;n++){
                double newval = stateSolutions[i][n]-previousConfig[n];
                if(n==2){
                    if(newval<0){
                        newval=-newval;
                    }
                }else{
                    if(newval>3.14)newval-=6.28;
                    if(newval<-3.14) newval+=6.28;
                }
                
                sum+=newval*newval;
                //std::cout<<newval<<" ";
            }
            //std::cout<<std::endl;
            
            double dist=std::sqrt(sum);//(stateSolutions[i]-previousConfig).norm2();
            if(dist<bestDist){


                bestDist=dist;
                bestSolution=i;
            }
            
        }
    }
    
    //std::cout <<"Collision at:"<< pos<<" Angle: "<<rot<<std::endl;
    //RW_THROW ("Collision on point... check path!");
    robotUR5->setQ (stateSolutions[bestSolution], state);
    //std::cout<<"Best dist: "<<bestDist<<std::endl; 

    return stateSolutions[bestSolution];
}

std::vector<Q> getCircledPoints(WorkCell::Ptr wc, State& state, rw::proximity::CollisionDetector& detector, 
MovableFrame::Ptr cylinderFrame, SerialDevice::Ptr robotUR5, Vector3D<> start, Vector3D<> end){

    Vector3D<> focal = Vector3D<>(0,0,-0.1);//-((start+end)*0.5);
    //focal+=Vector3D<>(0,0,-0.1);

    Vector3D<> dir=(end-start)/5.0;

    std::vector<Vector3D<>> points;

    double desiredLenght = ((focal-start).norm2()+(focal-end).norm2())*0.5;


    std::vector< Q > qs;
    points.push_back(start);
    for(int i=1;i<=0;i++){
        Vector3D<> newPoint=start+(dir*i);

        Vector3D<> newDir=newPoint-focal;
        double lenght = newDir.norm2();

        double newLenght= desiredLenght*1.3;

        newDir= newDir.normalize();
        newPoint=focal+(newDir*newLenght);
        points.push_back(newPoint);


    }
    points.push_back(end);

    Q lastDist (6, -2.234, -1.004, 1.655, -0.389, -0.68, -0.205);

    for(int i=0;i<2;i++){
        double angle = std::atan2 ((double)points[i][0],(double)points[i][1])-1.6;

        qs.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,points[i],angle,lastDist));

        lastDist=qs[i];
    }


    return qs;



}


int main (int argc, char** argv)
{


    


    

    // load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load ("../../WorkCell/Scene.wc.xml");
    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    // find Device
    SerialDevice::Ptr robotUR5 = wc->findDevice< rw::models::SerialDevice > ("UR-6-85-5-A");
    if (robotUR5.isNull ()) {
        RW_THROW ("COULD not find device UR5 ... check model");
    }

    MovableFrame::Ptr cylinderFrame = wc->findFrame< MovableFrame > ("Bottle");
    if (cylinderFrame.isNull ()) {
        RW_THROW ("COULD not find movable frame Cylinder ... check model");
    }

    Frame* tool_frame   = wc->findFrame ("GraspTCP");

    Frame* world_frame   = wc->findFrame ("WORLD");
    

    State state = wc->getDefaultState ();

    rw::proximity::CollisionDetector detector (wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    

    // define movement
    Q start (6, -2.234, -1.004, 1.655, -0.389, -0.68, -0.205);
    Q end (6, 1.508, -1.28, 1.784, -1.097, -0.088, 0.588);
    std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();





    bool doGribFromAbove=true;
    bool doLinearInterp=true;
    for(int i = 0; i<4;i++){
        doGribFromAbove=true;
        doLinearInterp=true;
        //i=3;
        if(i == 1 || i == 3) doGribFromAbove=false;
        if(i>=2) doLinearInterp=false;

        std::ofstream cubicPositions;
        cubicPositions.open ("../../Output/DKPos"+std::to_string(i)+".txt");

        std::ofstream timeTaken;
        timeTaken.open ("../../Output/linearTime.txt");

        if (cubicPositions.is_open() )
        {
            
        }
        else std::cout << "Unable to open file";
        

        std::cout<<i<<std::endl;
        double x1 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double y1 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double x2 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double y2 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double x1Width=0.7;
        double y1Width=0.2;
        double x2Width=0.15;
        double y2Width=0.15;
        x1=(x1*x1Width)-(x1Width*0.5);
        y1=(y1*y1Width)-(y1Width*0.5);
        x2=(x2*x2Width)-(x2Width*0.5);
        y2=(y2*y2Width)-(y2Width*0.5);
        


        //Vector3D<> startPos(0.0+x1,0.45+y1,0.111);
        //Vector3D<> endPos(0.3+x2,-0.5+y2,0.111);

        Vector3D<> restPos(0.433,0.120,0.111);
        Vector3D<> prepareGripPos(-0.081,0.455,0.161);
        Vector3D<> pickPos(-0.156,0.455,0.111);
        Vector3D<> clearencePos(-0.156,0.455,0.311);
        Vector3D<> clearencePosAbove(-0.15,0.455,0.211);
        Vector3D<> endPos(0.3,-0.475,0.116);
        Vector3D<> clearenceEndPos(0.2,-0.475,0.116);

        

        Kinematics::gripFrame (cylinderFrame, tool_frame, state);

        std::vector< Q > points;//= getCircledPoints(wc, state, detector,cylinderFrame, robotUR5,Vector3D<>(0.0, 0.45, 0.110),Vector3D<>(0.3, -0.5, 0.110));


        if(!doGribFromAbove){

            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,restPos,3.14 ,start,doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,prepareGripPos,-1.543  ,points.back(),doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,pickPos,-1.543   ,points.back(),doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,clearencePos,-1.543   ,points.back(),doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,endPos,1.543   ,points.back(),doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,clearenceEndPos,1.543    ,points.back(),doGribFromAbove));
        }else{
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,restPos,3.14 ,start,doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,prepareGripPos,-1.543  ,points.back(),doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,pickPos,-1.543   ,points.back(),doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,clearencePosAbove,5   ,points.back(),doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,endPos,1.543   ,points.back(),doGribFromAbove));
            points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,clearenceEndPos,1.543    ,points.back(),doGribFromAbove));
        }
        //points.push_back(getConfigFromPos(wc, state, detector,cylinderFrame, robotUR5,restPos,3.14  ,points.back()));

        TimedStatePath linearQMotion;
        //std::cout<<points[0];
        if(doLinearInterp){

            linearQMotion = linInterp (robotUR5, wc->getDefaultState (),cylinderFrame,tool_frame, world_frame , points, 5, cubicPositions, detector,doGribFromAbove);
        }else{
            linearQMotion = quadInterp(robotUR5, wc->getDefaultState (),cylinderFrame,tool_frame, world_frame , points, 5, cubicPositions, detector,doGribFromAbove);
        }
        if(linearQMotion.size()<=2){
            i--;
        }else{

            PathLoader::storeTimedStatePath (*wc, linearQMotion, "../../WorkCell/visu"+ std::to_string(i)+".rwplay");
            std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
            timeTaken<<(std::chrono::duration_cast<std::chrono::milliseconds>(endTime - beginTime).count())*0.001<<" ";
            std::cout << "Time Taken = " << (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - beginTime).count())*0.001 << "[s]" << std::endl;
            beginTime = std::chrono::steady_clock::now();


        }
        
        cubicPositions.close();
    }


    

    
    

}