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

#include "../RTKLIB/src/rtklib.h"

#include <stdarg.h>
#include <ros/ros.h>
#include <stdio.h>
#include <assert.h>

extern void postposRegisterPub(ros::NodeHandle &n);
extern void rtkposRegisterPub(ros::NodeHandle &n);
extern void pntposRegisterPub(ros::NodeHandle &n);

bool checkFile(const std::string &ParamName, char * FileNamePointer)
{
    FILE *file;
    std::string FileName;
    const bool ParamExists = ros::param::has(ParamName);
    if (!ParamExists)
    {
        ROS_INFO("\033[31m----> No parameter %s provided. Stopping!\033[0m", ParamName.c_str());
        return false;
    }
    else
    {
        ros::param::get(ParamName, FileName);
        if (file = fopen(FileName.c_str(), "r")) 
        {
            fclose(file);
        } 
        else 
        {
            ROS_INFO("%s: %s", ParamName.c_str() ,FileName.c_str());
            ROS_INFO("\033[31m----> File %s not found. Stopping!\033[0m", ParamName.c_str());
            return false;
        }
    }
    
    ROS_INFO("%s: %s", ParamName.c_str() ,FileName.c_str());
    
    /* copy to pointer */
    strcpy(FileNamePointer, strdup(FileName.c_str())); 
    
    return true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gnss_preprocessor_node");
    ros::NodeHandle nh("~");
    ROS_INFO("\033[1;32m----> gnss_preprocessor Started.\033[0m");
    
    /* input node handle */
    postposRegisterPub(nh);
    rtkposRegisterPub(nh);
    pntposRegisterPub(nh);

    /* get setup parameters from yaml config */
    int mode, nf, soltype, elevationmask;
    std::vector<std::string> satellites;
    bool shared_ephemeris, precise_ephemeris, ionex_correction, custom_atx;
    nh.getParam("/satellites", satellites);
    nh.param("/shared_ephemeris", shared_ephemeris, false);
    nh.param("/precise_ephemeris", precise_ephemeris, false);
    nh.param("/mode",   mode, 2);
    nh.param("/nf",     nf, 2);
    nh.param("/soltype",soltype, 2);
    nh.param("/elevationmask",elevationmask, 0);
    nh.param("/ionex_correction",ionex_correction, true);
    nh.param("/custom_atx",custom_atx, false);
    
    /* load option structs*/
    prcopt_t prcopt = prcopt_default;   // processing option
    solopt_t solopt = solopt_default;   // output solution option
    filopt_t filopt = {""};             // file option
    char *infile[10];                   // input files
    
    /* set processing options*/
    prcopt.mode = mode;                 // Positioning mode
    prcopt.navsys = 0;                  // all used satellites
    prcopt.nf = nf;                     // frequency (1:L1,2:L1+L2,3:L1+L2+L5)
    prcopt.soltype = soltype;           // 0:forward,1:backward,2:combined
    prcopt.elmin = elevationmask * D2R; // elevation mask (rad)
    prcopt.tidecorr = 0;                // earth tide correction (0:off,1-:on)
    prcopt.posopt[4] = 0;               // use RAIM FDE (qmo)  1
    prcopt.tropopt = TROPOPT_SAAS;      // default troposphere correction
    prcopt.ionoopt = IONOOPT_BRDC;      // default ionosphere correction
    prcopt.sateph = EPHOPT_BRDC;        // default ephemeris
    prcopt.modear = 3;                  // AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold)
    
    /* if you use the RTK mode, specify the position of the station (only used by RTKLIB)
     * following is an example position of the base HKSC in Hong Kong */
    prcopt.rb[0] = -2414266.9197;           // base position for relative mode {x,y,z} (ecef) (m)
    prcopt.rb[1] = 5386768.9868;            // base position for relative mode {x,y,z} (ecef) (m)
    prcopt.rb[2] = 2407460.0314;            // base position for relative mode {x,y,z} (ecef) (m)

    /* read paths for datasets specified in the launchfile */
    int n = 0;    
    
    /* set input files */
    char infile_[10][1024]={""};
    for (int i = 0; i < 10; i++) 
    {
            infile[i]=infile_[i];
    }
    
    /* observations of the rover */
    if(!checkFile("roverMeasureFile", infile[n++]))
    {
        return 0;  
    }

    /* observations of the base station */
    if(mode != 0)
    {
        // if the mode is set to 0 (single) there is no base station
        if(!checkFile("baseMeasureFile", infile[n++]))
        {
             return 0;   
        }   
    }
    
    /* read shared ephemeris file */
    if(shared_ephemeris)
    {
        if(!checkFile("EmpFile", infile[n++]))
        {
             return 0;   
        }
    }
    
    /* GPS ephemeris */
    if (std::find(satellites.begin(), satellites.end(), "GPS") != satellites.end())
    {
        if(!shared_ephemeris)
        {
            if(!checkFile("GPSEmpFile", infile[n++]))
            {
                 return 0;   
            }
        }
        prcopt.navsys = prcopt.navsys + SYS_GPS; 
    }
    
    /* GLONASS ephemeris */
    if (std::find(satellites.begin(), satellites.end(), "GLONASS") != satellites.end())
    {
        if(!shared_ephemeris)
        {
            if(!checkFile("GLONASSEmpFile", infile[n++]))
            {
                 return 0;   
            }
        }
        prcopt.navsys = prcopt.navsys + SYS_GLO; 
    }
    
    /* BeiDou ephemeris */
    if (std::find(satellites.begin(), satellites.end(), "BeiDou") != satellites.end())
    {
        if(!shared_ephemeris)
        {
            if(!checkFile("BeiDouEmpFile", infile[n++]))
            {
                 return 0;   
            }
        }
        prcopt.navsys = prcopt.navsys + SYS_CMP; 
    }
    
    /* Galileo ephemeris */
    if (std::find(satellites.begin(), satellites.end(), "Galileo") != satellites.end())
    {
        if(!shared_ephemeris)
        {
            if(!checkFile("GalileoEmpFile", infile[n++]))
            {
                 return 0;   
            }
        }
        prcopt.navsys = prcopt.navsys + SYS_GAL; 
    }
    
    /* QZSS ephemeris */
    if (std::find(satellites.begin(), satellites.end(), "QZSS") != satellites.end())
    {
        if(!shared_ephemeris)
        {
            if(!checkFile("QZSSEmpFile", infile[n++]))
            {
                 return 0;   
            }
        }
        prcopt.navsys = prcopt.navsys + SYS_QZS; 
    }
    
    /* SBAS Payload */
    if (std::find(satellites.begin(), satellites.end(), "SBAS") != satellites.end())
    {
        if(!checkFile("SBASFile", infile[n++]))
        {
             return 0;   
        }
        prcopt.navsys = prcopt.navsys + SYS_SBS;
        prcopt.tropopt = TROPOPT_SBAS;  
        prcopt.ionoopt = IONOOPT_SBAS;
    }

    /* precise ephemeris and clock */
    if (precise_ephemeris)
    {
        if(!checkFile("SP3File", infile[n++]))
        {
             return 0;   
        }
        if(!checkFile("clkFile", infile[n++]))
        {
             n--;
        }
        prcopt.sateph = EPHOPT_PREC;
    }

    /* precise ionospheric corrections */
    if (ionex_correction)
    {
        char buffer[1024];
        if(!checkFile("ionexFile", &buffer[0]))
        {
             return 0;   
        }
        strcpy(filopt.iono, buffer);
        prcopt.ionoopt = IONOOPT_TEC;
    }
    
    /* a custom antenna file */
    if (custom_atx)
    {
        char buffer[1024];
        if(!checkFile("atxFile", &buffer[0]))
        {
             return 0;   
        }
        strcpy(filopt.satantp, buffer);
        strcpy(filopt.rcvantp, buffer);
    }
    
    /* set output file */
    char outfile[1024];
    std::string out_folder;
    if(!ros::param::get("out_folder", out_folder))
    {
        return 0;
    }
    strcpy(outfile, strdup(out_folder.c_str()));


    /* flag for state */
    int stat;

    /* processing time setting */
    double ti=0.0;                      // processing interval  (s) (0:all)
    double tu=0.0;                      // unit time (s) (0:all)
    gtime_t ts={0},te={0};
    ts.time=0;                          // start time (0:all)
    te.time=0;                          // end time (0:all)

    solopt.outopt = 1;                  // output processing options (0:no,1:yes)
    solopt.timef = 0;                   // time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s)
    solopt.timeu = 3;                   // time digits under decimal point
    solopt.sep[0] = ',';                // field separator
    solopt.sstat= 0;                    // solution statistics level (0:off,1:states,2:residuals)
    solopt.trace = 0;                   // debug trace level (0:off,1-5:debug)
    solopt.sstat = 0;                   // get the solution file
    solopt.posf = SOLF_LLH;
    solopt.height = 0;
    
    // rover and base id
    char rov[] = "rover";
    const char base[] = "base";

    /* decode the RINEX file positioning */
    stat=postpos(ts,te,ti,tu,&prcopt,&solopt,&filopt,infile,n,outfile,&rov[0],&base[0]);

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
