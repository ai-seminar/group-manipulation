#include <../include/iai_seminar_manipulation_utils/ParameterServerUtils.h>
#include <ros/ros.h>
#include <vector>
#include <XmlRpcValue.h>

using namespace ros;
using namespace std;

/** Loads a vector of strings from the parameter server.

    \param n [in] namespace to use for the lookup.
    \param parameter_name [in] name of the parameter holding the vector.
    \param parameters [out] container for loaded vector.
    \return [out] true if everything worked out fine, else false.
*/
bool loadStringVectorFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  std::vector<std::string>& parameters){
	XmlRpc::XmlRpcValue a;
	for (int i = 0; i<7; ++i){
		if (!n.getParam(n.getNamespace()+parameter_name, a)) return false;
		parameters.push_back(a[i]);
	}
	return true;
}

/** Loads a vector of doubles from the parameter server.

    \param n [in] namespace to use for the lookup.
    \param parameter_name [in] name of the parameter holding the vector.
    \param parameters [out] container for loaded vector.
    \return [out] true if everything worked out fine, else false.
*/
bool loadDoubleVectorFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  std::vector<double>& parameters){
	XmlRpc::XmlRpcValue a;
	for (int i = 0; i<7; ++i){
		if (!n.getParam(n.getNamespace()+parameter_name, a)) return false;
		parameters.push_back(a[i]);
	}
	return true;
}	  

/** Loads a single double from the parameter server.

    \param n [in] namespace to use for the lookup.
    \param parameter_name [in] name of the parameter.
    \param parameter [out] container for loaded double.
    \return [out] true if everything worked out fine, else false.
*/
bool loadDoubleFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  double& parameter){
	return (!n.getParam(n.getNamespace()+parameter_name, parameter));
}	  
