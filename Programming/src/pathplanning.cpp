#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <streambuf>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 100.

void exportLUAScript(ostringstream &finalstream)
{
    string finalstring = finalstream.str();
    ifstream tt("lua_template.txt");
    stringstream buffer;
    buffer << tt.rdbuf();
    string buffer2 = buffer.str();
    ofstream myfile;
    myfile.open("lua.txt");
    myfile << buffer2 << endl << finalstring << endl;
    myfile.close();
}


bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data); // this has to be true in order to be collision free
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}


int main(int argc, char** argv) {
    // Configure Random Seed
    rw::math::Math::seed();

    // Loading the Workcell
    const string wcFile = "/home/student/Desktop/Robotics/MandatoryExercise2/Kr16WallWorkCell/Scene.wc.xml"; // Path to Workcell
	const string deviceName = "KukaKr16";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}


    // Getting the default state
    State state = wc->getDefaultState();

    // Setup a default collision detector and planner Constraint
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	/** Most easy way: uses default parameters based on given device
		sampler: QSampler::makeUniform(device)
		metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
		extend: 0.05 */
    //QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device, RRTPlanner::RRTConnect);

	/** More complex way: allows more detailed definition of parameters and methods */
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    double extend = 0.15;
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    //default configuration
    //Q from(6,-0.2,-0.6,1.5,0.0,0.6,1.2);
    //Q to(6,1.4,-1.3,1.5,0.3,1.3,1.6);

    //Q to(6,1.7,0.6,-0.8,0.3,0.7,-0.5); // Very difficult for planner

    //custom configuration
    Q from(6, -3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
    //Q from(6,-0.2,-0.6,1.5,0.0,0.6,1.2);

    //Q from(6,-0.2,-0.6,1.5,0.0,0.6);

    //Q to(6,1.4,-1.3,1.5,0.3,1.3);
    Q to(6, 1.571, 0.006, 0.03, 0.153, 0.762, 4.49);

    device->setQ(from, state);
    rw::kinematics::Kinematics::gripFrame(wc->findFrame("Bottle"), wc->findFrame("Tool"), state);
    constraint = PlannerConstraint::make(&detector,device,state);
    sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);



    if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;
    cout << "test3" << endl;
	cout << "Planning from " << from << " to " << to << endl;

    QPath path;
    QPath shortestPath;
    int shortestPathLength = 0xffffffff;
    Timer t;
    int iterations = 10;
    int longestPathLength = 0;
    double mean;


    for(int i = 0; i < iterations; i++){
        //QPath *pointerToPath = &path;
        path.clear();
        t.resetAndResume();
        planner->query(from,to,path,MAXTIME);
        t.pause();
        cout << "(" << setfill('0') << setw(3) << i+1 << "/" << iterations << ") Path of length " << path.size() << " found in " << t.getTime() << " seconds."  << endl;
        if (t.getTime() >= MAXTIME) {
            cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
        }

        if(path.size() < shortestPathLength){
            shortestPathLength = path.size();
            shortestPath = path;
        }
        if(path.size() > longestPathLength){
            longestPathLength = path.size();
        }
        mean += path.size();
    }
    mean /= iterations;

    ostringstream finalstream;
    for (QPath::iterator it = shortestPath.begin(); it < shortestPath.end(); it++) {
        ostringstream stream;
        stream << *it;
        string str =  stream.str();
        str.replace(0, 4, "setQ(");
        str.push_back(')');
        cout << str << endl;
        finalstream << str << endl;
	}
    cout << endl << "Statistics for extend-value " << extend << ": " << endl;
    cout << "Mean of Path Lengths: " << mean << endl;

    cout << "The longest  Path contains " << longestPathLength << " steps!" << endl;
    cout << "The shortest Path contains " << shortestPathLength << " steps!" << endl;


    // Call the export function to create the final lua code
    exportLUAScript(finalstream);

	cout << "Program done." << endl;
	return 0;
}



