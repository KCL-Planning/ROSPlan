
/***************************************************************************
 *  print_plan.cpp - print plan as PDDL to output
 *
 *  Created: Wed Mar 22 14:40:56 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/CompletePlan.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>

#include <cstdio>

void
plan_cb(const rosplan_dispatch_msgs::CompletePlan::ConstPtr& msg)
{
	for (int i=0; i<msg->plan.size(); i++) {
		std::string s;
		for (int j=0; j<msg->plan[i].parameters.size(); j++) {
			s += " " + msg->plan[i].parameters[j].value;
		}
		printf("%7.3f: (%s %s)  [%.3f]\n", msg->plan[i].dispatch_time, msg->plan[i].name.c_str(), s.c_str(), msg->plan[i].duration);
	}
	ros::shutdown();
}




int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rcll_refbox_peer");

	ros::NodeHandle n;
	ros::Subscriber sub_plan = n.subscribe("kcl_rosplan/plan", 1, plan_cb);

	ros::spin();
	
	return 0;
}
