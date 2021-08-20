/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * Function: decode the RINEX file, output GNSS raw measurements via ros topics
 * Date: 2020/11/27
 *******************************************************/

#include <stdarg.h>
#include "../../RTKLIB/src/rtklib.h"
#include <ros/ros.h>
#include <stdio.h>
#include <assert.h>

extern void postposRegisterPub(ros::NodeHandle &n);
extern void rtkposRegisterPub(ros::NodeHandle &n);
extern void pntposRegisterPub(ros::NodeHandle &n);

int main(int argc, char **argv)
{
	
    ros::init(argc, argv, "gnss_preprocessor_node");
	ros::NodeHandle nh("~");
	ROS_INFO("\033[1;32m----> gnss_preprocessor Started.\033[0m");

	/* get setup parameters */
	int mode, nf, soltype, elevationmask;
	std::vector<std::string> satellites;
	nh.getParam("/satellites", satellites);
	nh.param("/mode",   mode, 2);
	nh.param("/nf",     nf, 2);
	nh.param("/soltype",soltype, 2);
	nh.param("/elevationmask",elevationmask, 0);

	std::string roverMeasureFile, baseMeasureFile, BeiDouEmpFile, GPSEmpFile, GLONASSEmpFile, GalileoEmpFile, SP3file, SBASfile;
	ros::param::get("roverMeasureFile", roverMeasureFile);
	if(mode != 0){	// if the mode is set to 0 (single) there is no base station
		ros::param::get("baseMeasureFile", baseMeasureFile);
	}
	if (std::find(satellites.begin(), satellites.end(), "BeiDou") != satellites.end()){	// find "BeiDou" in the satellite list
		ros::param::get("BeiDouEmpFile", BeiDouEmpFile);
	}
	if (std::find(satellites.begin(), satellites.end(), "GPS") != satellites.end()){
		ros::param::get("GPSEmpFile", GPSEmpFile);
	}
	if (std::find(satellites.begin(), satellites.end(), "GLONASS") != satellites.end()){
		ros::param::get("GLONASSEmpFile", GLONASSEmpFile);
	}
	if (std::find(satellites.begin(), satellites.end(), "Galileo") != satellites.end()){
		ros::param::get("GalileoEmpFile", GalileoEmpFile);
	}
	if (std::find(satellites.begin(), satellites.end(), "SP3") != satellites.end()){
		ros::param::get("SP3file", SP3file);
	}
	if (std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end()){
		ros::param::get("SBASfile", SBASfile);
	}
	std::string out_folder;
	ros::param::get("out_folder", out_folder);

	/* flag for state */
    int n=0,i,stat;

	/* input node handle */
	postposRegisterPub(nh);
	rtkposRegisterPub(nh);
	pntposRegisterPub(nh);

	/* processing time setting */
	double ti=0.0;						// processing interval  (s) (0:all)
	double tu=0.0;						// unit time (s) (0:all)
	gtime_t ts={0},te={0};
	ts.time=0;							// start time (0:all)
	te.time=0;							// end time (0:all)

	/* options */
	prcopt_t prcopt = prcopt_default;	// processing option
	solopt_t solopt = solopt_default;	// output solution option
	filopt_t filopt = {""};	            // file option
	prcopt.mode = mode;			// Kinematic RTK
	prcopt.navsys = 0;			// all used satellites
	if (std::find(satellites.begin(), satellites.end(), "BeiDou") != satellites.end()){
		prcopt.navsys = prcopt.navsys + SYS_CMP;
	}
	if (std::find(satellites.begin(), satellites.end(), "GPS") != satellites.end()){
		prcopt.navsys = prcopt.navsys + SYS_GPS;
	}
	if (std::find(satellites.begin(), satellites.end(), "GLONASS") != satellites.end()){
		prcopt.navsys = prcopt.navsys + SYS_GLO;
	}
	if (std::find(satellites.begin(), satellites.end(), "Galileo") != satellites.end()){
		prcopt.navsys = prcopt.navsys + SYS_GAL;
	}
	if (std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end()){
		prcopt.navsys = prcopt.navsys + SYS_SBS;
	}
	prcopt.nf = nf;						// frequency (1:L1,2:L1+L2,3:L1+L2+L5) 
	prcopt.soltype = soltype;					// 0:forward,1:backward,2:combined
	prcopt.elmin = elevationmask*D2R;				// elevation mask (rad)	
	prcopt.tidecorr = 0;					// earth tide correction (0:off,1-:on) 
	prcopt.posopt[4] = 0;               // use RAIM FDE (qmo)  1
	prcopt.tropopt = TROPOPT_SAAS;        // troposphere option: Saastamoinen model
	prcopt.ionoopt = IONOOPT_BRDC;		// ionosphere option: Broad cast
	if (std::find(satellites.begin(), satellites.end(), "SP3") != satellites.end() or std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end()){
		prcopt.sateph = EPHOPT_PREC;			// ephemeris option: broadcast ephemeris (0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom)
	}
	else{
		prcopt.sateph = EPHOPT_BRDC;			// ephemeris option: broadcast ephemeris (0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom)
	}

	prcopt.modear = 3;					// AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold)

	solopt.outopt = 1;					// output processing options (0:no,1:yes)
	solopt.timef = 0;						// time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s)
	solopt.timeu = 3;						// time digits under decimal point
	solopt.sep[0] = ',';					// field separator
	solopt.sstat= 0;						// solution statistics level (0:off,1:states,2:residuals)
	solopt.trace = 0;						// debug trace level (0:off,1-5:debug)
	solopt.sstat = 0;						// get the solution file
	solopt.posf = SOLF_LLH;
	solopt.height = 0;

	char *rov="",*base="";
	char infile_[10][1024]={""}, *infile[10];
	char outfile[1024];

	/* set input files */
	for (i=0;i<10;i++) infile[i]=infile_[i];

	strcpy(infile[n++],strdup(roverMeasureFile.c_str()));
	if(mode != 0){
		strcpy(infile[n++],strdup(baseMeasureFile.c_str()));
	}
	if (std::find(satellites.begin(), satellites.end(), "BeiDou") != satellites.end()){
		strcpy(infile[n++],strdup(BeiDouEmpFile.c_str()));
	}
	if (std::find(satellites.begin(), satellites.end(), "GPS") != satellites.end()){
		strcpy(infile[n++],strdup(GPSEmpFile.c_str()));
	}
	if (std::find(satellites.begin(), satellites.end(), "GLONASS") != satellites.end()){
        	strcpy(infile[n++],strdup(GLONASSEmpFile.c_str()));
	}
	if (std::find(satellites.begin(), satellites.end(), "Galileo") != satellites.end()){        
		strcpy(infile[n++],strdup(GalileoEmpFile.c_str()));
	}
	if (std::find(satellites.begin(), satellites.end(), "SP3") != satellites.end()){
        	strcpy(infile[n++],strdup(SP3file.c_str()));
	}
	if (std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end()){
        	strcpy(infile[n++],strdup(SBASfile.c_str()));
	}

	/* if you use the RTK mode, specify the position of the station (only used by RTKLIB)
	 * following is an example position of the base HKSC in Hong Kong */
	prcopt.rb[0] = -2414266.9197;			// base position for relative mode {x,y,z} (ecef) (m)
	prcopt.rb[1] = 5386768.9868;			// base position for relative mode {x,y,z} (ecef) (m)
	prcopt.rb[2] = 2407460.0314;			// base position for relative mode {x,y,z} (ecef) (m)

	/* set output files */
	strcpy(outfile, strdup(out_folder.c_str()));

	/* decode the RINEX files*/
	// while(ros::ok())
	{
		/* decode the RINEX file positioning */
		stat=postpos(ts,te,ti,tu,&prcopt,&solopt,&filopt,infile,n,outfile,rov,base);

		printf("\n");
		if(stat==0){
			ROS_INFO("\033[1;32m----> gnss_preprocessor Finished.\033[0m");
		}
		else if(stat>0){
			ROS_INFO("\033[1;32m----> gnss_preprocessor Error!!!.\033[0m");
		}
		else if(stat==-1){
			ROS_INFO("\033[1;32m----> gnss_preprocessor Aborted!!!.\033[0m");
		}
		// system("pause");
	}
	// while(ros::ok())
	// {}
    ros::spin();
    return 0;
}

/*****dummy application functions for shared library*****/
extern int showmsg(char *format,...) {
	va_list arg;
	char buff[1024];
	if (*format) {
		va_start(arg,format);
		vsprintf(buff,format,arg);
		va_end(arg);
		printf("%s\n",buff);
	}
	return 0;	
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}
