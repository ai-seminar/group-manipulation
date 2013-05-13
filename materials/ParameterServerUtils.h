/*  Copyright (c) 2013, Georg Bartels (georg.bartels@cs.uni-bremen.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *   
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the Institute for Artificial Intelligence/Universität Bremen
 *      nor the names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PARAMETERSERVERUTILS_H_
#define PARAMETERSERVERUTILS_H_

#include <ros/ros.h>
#include <vector>

/** Loads a vector of strings from the parameter server.

    \param n [in] namespace to use for the lookup.
    \param parameter_name [in] name of the parameter holding the vector.
    \param parameters [out] container for loaded vector.
    \return [out] true if everything worked out fine, else false.
*/
bool loadStringVectorFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  std::vector<std::string>& parameters);

/** Loads a vector of doubles from the parameter server.

    \param n [in] namespace to use for the lookup.
    \param parameter_name [in] name of the parameter holding the vector.
    \param parameters [out] container for loaded vector.
    \return [out] true if everything worked out fine, else false.
*/
bool loadDoubleVectorFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  std::vector<double>& parameters);

/** Loads a single double from the parameter server.

    \param n [in] namespace to use for the lookup.
    \param parameter_name [in] name of the parameter.
    \param parameter [out] container for loaded double.
    \return [out] true if everything worked out fine, else false.
*/
bool loadDoubleFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  double& parameter);

#endif /* PARAMETERSERVERUTILS_H_ */
