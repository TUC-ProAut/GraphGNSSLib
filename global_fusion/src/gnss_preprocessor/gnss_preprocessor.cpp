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

	/* get setup parameters from yaml config */
	int mode, nf, soltype, elevationmask, ionosphere_correction;
	std::vector<std::string> satellites;
	bool precise_ephemeris;
	nh.getParam("/satellites", satellites);
	nh.param("/precise_ephemeris", precise_ephemeris, false);
	nh.param("/mode",   mode, 2);
	nh.param("/nf",     nf, 2);
	nh.param("/soltype",soltype, 2);
	nh.param("/elevationmask",elevationmask, 0);
	nh.param("/ionosphere_correction",ionosphere_correction, 0);

	/* read paths for datasets specified in the launchfile */
	FILE *file;
	std::string roverMeasureFile, baseMeasureFile, BeiDouEmpFile, GPSEmpFile, GLONASSEmpFile, GalileoEmpFile, QZSSEmpFile, SP3file, SBASfile, ionexFile;
	if (! ros::param::get("roverMeasureFile", roverMeasureFile)){
			ROS_INFO("\033[31m----> No Rover file provided. Stopping!\033[0m");
			return 0;
		}
	else{
		if (file = fopen(roverMeasureFile.c_str(), "r")) {
			fclose(file);
		} else {
			ROS_INFO("\033[31m----> Rover file not found. Stopping!\033[0m");
			return 0;
		}
	}
	if(mode != 0){	// if the mode is set to 0 (single) there is no base station
		if (! ros::param::get("baseMeasureFile", baseMeasureFile)){
			ROS_INFO("\033[31m----> No Base file provided. Stopping!\033[0m");
			return 0;
		}
		else{
			if (file = fopen(baseMeasureFile.c_str(), "r")) {
				fclose(file);
			} else {
				ROS_INFO("\033[31m----> Base file not found. Stopping!\033[0m");
				return 0;
			}
		}
	}
	if (std::find(satellites.begin(), satellites.end(), "BeiDou") != satellites.end()){	// find "BeiDou" in the satellite list
		if (! ros::param::get("BeiDouEmpFile", BeiDouEmpFile)){
			ROS_INFO("\033[31m----> No BeiDou file provided. Stopping!\033[0m");
			return 0;
		}
		else{
			if (file = fopen(roverMeasureFile.c_str(), "r")) {
				fclose(file);
			} else {
				ROS_INFO("\033[31m----> BeiDou file not found. Stopping!\033[0m");
				return 0;
			}
		}
	}
	if (std::find(satellites.begin(), satellites.end(), "GPS") != satellites.end()){
		if (! ros::param::get("GPSEmpFile", GPSEmpFile)){
			ROS_INFO("\033[31m----> No GPS file provided. Stopping!\033[0m");
			return 0;
		}
		else{
			if (file = fopen(GPSEmpFile.c_str(), "r")) {
				fclose(file);
			} else {
				ROS_INFO("\033[31m----> GPS file not found. Stopping!\033[0m");
				return 0;
			}
		}
	}
	if (std::find(satellites.begin(), satellites.end(), "GLONASS") != satellites.end()){
		if (! ros::param::get("GLONASSEmpFile", GLONASSEmpFile)){
			ROS_INFO("\033[31m----> No GLONASS file provided. Stopping!\033[0m");
			return 0;
		}
		else{
			if (file = fopen(GLONASSEmpFile.c_str(), "r")) {
				fclose(file);
			} else {
				ROS_INFO("\033[31m----> GLONASS file not found. Stopping!\033[0m");
				return 0;
			}
		}
	}
	if (std::find(satellites.begin(), satellites.end(), "Galileo") != satellites.end()){
		if (! ros::param::get("GalileoEmpFile", GalileoEmpFile)){
			ROS_INFO("\033[31m----> No Galileo file provided. Stopping!\033[0m");
			return 0;
		}
		else{
			if (file = fopen(GalileoEmpFile.c_str(), "r")) {
				fclose(file);
			} else {
				ROS_INFO("\033[31m----> Galileo file not found. Stopping!\033[0m");
				return 0;
			}
		}
	}
	if (std::find(satellites.begin(), satellites.end(), "QZSS") != satellites.end()){
		if (! ros::param::get("QZSSEmpFile", QZSSEmpFile)){
			ROS_INFO("\033[31m----> No QZSS file provided. Stopping!\033[0m");
			return 0;
		}
		else{
			if (file = fopen(QZSSEmpFile.c_str(), "r")) {
				fclose(file);
			} else {
				ROS_INFO("\033[31m----> QZSS file not found. Stopping!\033[0m");
				return 0;
			}
		}
	}
	if (precise_ephemeris){
		if (! ros::param::get("SP3file", SP3file)){
			ROS_INFO("\033[31m----> No SP3 file provided. Stopping!\033[0m");
			return 0;
		}
		else{
			if (file = fopen(SP3file.c_str(), "r")) {
				fclose(file);
			} else {
				ROS_INFO("\033[31m----> SP3 file not found. Stopping!\033[0m");
				return 0;
			}
		}
	}
	if (std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end()){
		if (! ros::param::get("SBASfile", SBASfile)){
			ROS_INFO("\033[31m----> No SBAS file provided. Stopping!\033[0m");
			return 0;
		}
		else{
			if (file = fopen(SBASfile.c_str(), "r")) {
				fclose(file);
			} else {
				ROS_INFO("\033[31m----> SBAS file not found. Stopping!\033[0m");
				return 0;
			}
		}
	}
	if (ionosphere_correction == 5){
		if (! ros::param::get("ionexFile", ionexFile)){
			ROS_INFO("\033[31m----> No IONEX file provided. Stopping!\033[0m");
			return 0;
		}
		else{
			if (file = fopen(ionexFile.c_str(), "r")) {
				fclose(file);
			} else {
				ROS_INFO("\033[31m----> IONEX file not found. Stopping!\033[0m");
				return 0;
			}
		}
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
	prcopt.mode = mode;			// Positioning mode
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
	if (std::find(satellites.begin(), satellites.end(), "QZSS") != satellites.end()){
		prcopt.navsys = prcopt.navsys + SYS_QZS;
	}
	if (std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end() && (!precise_ephemeris)){	// only use SBAS if we do not use the precise (SP3) ephemeris data
		prcopt.navsys = prcopt.navsys + SYS_SBS;
	}
	prcopt.nf = nf;						// frequency (1:L1,2:L1+L2,3:L1+L2+L5)
	prcopt.soltype = soltype;					// 0:forward,1:backward,2:combined
	prcopt.elmin = elevationmask * D2R;				// elevation mask (rad)
	prcopt.tidecorr = 0;					// earth tide correction (0:off,1-:on)
	prcopt.posopt[4] = 0;               // use RAIM FDE (qmo)  1

	/* ephemeris and troposphere option */
	if (precise_ephemeris){
		prcopt.sateph = EPHOPT_PREC;			// ephemeris option: precise ephemeris
		if (std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end()){
			prcopt.tropopt = TROPOPT_SBAS;        		// troposphere option: SBAS model
		}
		else{
			prcopt.tropopt = TROPOPT_SAAS;        		// troposphere option: Saastamoinen model
		}
	}
	else if (std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end()){
		prcopt.sateph = EPHOPT_SBAS;			// ephemeris option: broadcast + sbas ephemeris
		prcopt.tropopt = TROPOPT_SBAS;        		// troposphere option: SBAS model
	}
	else{
		prcopt.sateph = EPHOPT_BRDC;			// ephemeris option: broadcast ephemeris
		prcopt.tropopt = TROPOPT_SAAS;        		// troposphere option: Saastamoinen model
	}

	// ionosphere option (0: off, 1: broadcast, 2: SBAS, 3: Iono-Free LC, 4: estimate TEC, 5: IONEX TEC, 6: QZSS broadcast)
	prcopt.ionoopt = ionosphere_correction;

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
	if (std::find(satellites.begin(), satellites.end(), "QZSS") != satellites.end()){
		strcpy(infile[n++],strdup(QZSSEmpFile.c_str()));
	}
	if (precise_ephemeris){
        strcpy(infile[n++],strdup(SP3file.c_str()));
	}
	if (std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end()){
        strcpy(infile[n++],strdup(SBASfile.c_str()));
	}
	if (ionosphere_correction == 5){
		strcpy(filopt.iono, ionexFile.c_str());	// copy the ionex path to the corresponding struct
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
