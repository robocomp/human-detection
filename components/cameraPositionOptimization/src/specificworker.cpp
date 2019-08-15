/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innermodel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	//initialize cameras
	std::vector<std::string> t_camera_names, camera_names;
	t_camera_names = {"cam1Translation", "cam2Translation", "cam3Translation"};
	camera_names = {"camera1", "camera2", "camera3"};
	
	cameras.push_back(QVec::vec6(-88.5111083984, -294.114929199, 3636.78735352, 0.00779854133725, -0.731414854527, 1.56042146683));
	cameras.push_back(QVec::vec6(217.058700562, -190.08354187, 4365.43603516, 0.881269991398, -0.0238653570414, -3.06511044502));
	cameras.push_back(QVec::vec6(-25.1116256714, 288.730499268, 4390.10351562, -0.775234341621, -0.0307027455419, -0.00459992652759));
	for(auto&& [name, t_name, cam] : iter::zip(camera_names, t_camera_names, cameras))
	{
		QVec t_pose = updateCameraPosition(t_name, cam);
		cameras_map[name] = std::tuple{t_name, t_pose, 
			new double[6] {t_pose[0]/1000., t_pose[1]/1000., t_pose[2]/1000., t_pose[3], t_pose[4], t_pose[5]}};
			//new double[6] {t_pose[0], t_pose[1], t_pose[2], t_pose[3], t_pose[4], t_pose[5]}};
	}

	// init compute
	this->Period = period;
	timer.setSingleShot(true);
	timer.start(Period);
}

void SpecificWorker::compute()
{
	createList();
	
	Problem problem;
	for(auto &&[cA, mA, cB, mB] : measurements)
	{	
		CostFunction* cost_function = new NumericDiffCostFunction<CostFunctor, ceres::RIDDERS, 3, 6, 6>
		  		(new CostFunctor(innermodel, cameras_map, cA, mA, cB, mB));
		
		double *mutA = std::get<double *>(cameras_map.at(cA));
		double *mutB = std::get<double *>(cameras_map.at(cB));
		
		problem.AddResidualBlock(cost_function, NULL, mutA, mutB);
	}
	// for(auto &&[cA, mA, cB, mB] : measurements)
	// {	
	// 	CostFunction* cost_function = new NumericDiffCostFunction<CostFunctor, ceres::RIDDERS, 3, 6>
	// 	  		(new CostFunctor(innermodel, cameras_map, cA, mA, cB, mB));
		
	// 	double *mutA = std::get<double *>(cameras_map.at(cA));
	// 	//double *mutB = std::get<double *>(cameras_map.at(cB));
		
	// 	problem.AddResidualBlock(cost_function, NULL, mutA);
	// }
	
	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 500;
	options.function_tolerance = 1E-8;
	//options.trust_region_strategy_type = ceres::DOGLEG;
	Solver::Summary summary;
	
	Solve(options, &problem, &summary);
	
	std::cout << summary.FullReport() << "\n";
	for(auto &&[name, val]  : cameras_map)
	{
		std::cout << name << std::endl;
		auto pose = std::get<QVec>(val);
		for(auto i: iter::range(6))
			std::cout << pose[i] << " ";
		std::cout << std::endl; 
		auto mut = std::get<double *>(val);
		for(auto i: iter::range(3))
			std::cout << mut[i]*1000. << " ";
			//std::cout << mut[i] << " ";
		for(auto i: iter::range(3,6))
			std::cout << mut[i] << " ";	
		std::cout << std::endl;
		std::cout << "----------------------" << std::endl;
	}
}

void SpecificWorker::createList()
{
	std::ifstream infile("april.txt");
	std::string line;
	std::list<std::tuple<std::string, QVec, std::string, QVec>> my_measurements;
	std::random_device r;
	std::default_random_engine e1(r());
	std::uniform_int_distribution<int> uniform_dist(0, 1);
	while (std::getline(infile, line))
	{ 
		QStringList list = QString::fromStdString(line).split(QRegExp("\\s+"), QString::SkipEmptyParts);
		 // Choose a random mean between 0 and 1
    	int choice = uniform_dist(e1);
		if(choice==0)
			my_measurements.push_back(std::tuple{ list[1].toStdString(), 
								   			   	  QVec::vec3(list[2].toDouble(), list[3].toDouble(), list[4].toDouble()),
											   	  list[8].toStdString(),
											   	  QVec::vec3(list[9].toDouble(), list[10].toDouble(), list[11].toDouble())});
		else
			my_measurements.push_back(std::tuple{ list[8].toStdString(), 
								   			   	  QVec::vec3(list[9].toDouble(), list[10].toDouble(), list[11].toDouble()),
											   	  list[1].toStdString(),
											   	  QVec::vec3(list[2].toDouble(), list[3].toDouble(), list[4].toDouble())});
								

	}
	qDebug() << "number of lines: "<< my_measurements.size();
	//const int ITEMS = 1000;
	const int ITEMS = my_measurements.size();
	// random resample
	std::sample(my_measurements.begin(), my_measurements.end(), std::back_inserter(this->measurements), ITEMS, std::mt19937{std::random_device{}()});
	qDebug() << "number of lines: "<< measurements.size();

	// check for outliers
	std::vector<double> residuals;
	for(auto &&[cA, mA, cB, mB] : measurements)
	{
		QVec rA = innermodel->transformS("world", mA, std::get<std::string>(cameras_map.at(cA)));
		QVec rB = innermodel->transformS("world", mB, std::get<std::string>(cameras_map.at(cB)));
		double res = (rA-rB).norm2();
		residuals.push_back(res);
	}
	std::sort(std::begin(residuals), std::end(residuals), std::greater<double>());
	double mean = std::accumulate(std::begin(residuals), std::end(residuals), 0.0) / residuals.size();
	std::vector<double> diff(residuals.size());
	std::transform(std::begin(residuals), std::end(residuals), diff.begin(), [mean](double x) { return x - mean; });
	double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	double stdev = std::sqrt(sq_sum / residuals.size());
	for(auto i : iter::range(300))
	 	std::cout << residuals[i] << std::endl;
	std::cout << mean << " " << stdev << std::endl;

	//filter
	auto cola = std::remove_if(std::begin(measurements), std::end(measurements), 
		[stdev, this](auto&& m)
			{ 
				auto&& [cA,mA,cB,mB] = m;
				//std::cout << cA << " " << cB << std::endl;
				QVec rA = innermodel->transformS("world", mA, std::get<std::string>(cameras_map.at(cA)));
				QVec rB = innermodel->transformS("world", mB, std::get<std::string>(cameras_map.at(cB)));
				double res = (rA-rB).norm2();
				return res > 3. * stdev;
			});	
	measurements.erase(cola, std::end(measurements));
	qDebug() << "Size" << measurements.size();
}
QVec SpecificWorker::updateCameraPosition(string camera, QVec values)
{
	Rot3DOX crx (-values.rx()); //***********RX inverse SIGN************
	Rot3DOY cry (values.ry());
	Rot3DOZ crz (values.rz());
	QMat crZYX = crz * cry * crx;

	RTMat cam = RTMat();
	cam.setR(crZYX);
	cam.setTr(values.x(), values.y(), values.z());
	cam = cam.invert();

	innermodel->getNode<InnerModelTransform>(camera)->setR(cam.getR());
	innermodel->getNode<InnerModelTransform>(camera)->setTr(cam.getTr());
	innermodel->cleanupTables();

	return innermodel->transformS6D("world", camera);		
}



