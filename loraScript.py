from collections import defaultdict
import os
import math
import random
import re
import subprocess
import sys
import argparse
from glob import glob
import numpy as np
import matplotlib.pylab as plt
from itertools import cycle
# To install YAML: sudo apt-get install python3-yaml
import yaml

class Simulation:
    def __init__(self, configurations_file):
        #self.email_to = 'sicrano@gmail.com'            
        with open(configurations_file, 'r') as f:
            self.doc = yaml.load(f, Loader=yaml.loader.BaseLoader)
        self.campaign_name = os.path.splitext(configurations_file)[0]
        
        # Read commom parameters
        self.campaignX = self.doc['scenario']['campaignX']
        self.campaignLines = self.doc['scenario']['campaignLines'][0]
        self.simLocation = str(doc['scenario']['simLocation'])
        self.simulationTime = self.doc['scenario']['simulationTime']
        self.nDevices = self.doc['scenario']['nDevices'][0]
        self.radius = self.doc['scenario']['radius'][0]
        self.appPeriodSeconds = self.doc['scenario']['appPeriodSeconds']
        self.bPrint = (self.doc['scenario']['bPrint'])
        self.fixedSeed = (self.doc['scenario']['fixedSeed'])
        self.algoritmo = self.doc['scenario']['algoritmo'][0]
        self.ns3_path = str(self.doc['scenario']['ns3_path'])
        #self.ns3_path = os.getcwd() + '/' + self.ns3_path
        self.ns3_path = self.ns3_path
        self.ns3_script = str(self.doc['scenario']['ns3_script'])
        self.nJobs = int(self.doc['scenario']['jobs'])
        self.targetRealocation  = (self.doc['scenario']['targetRealocation'])[0]
        self.filename = str(self.doc['scenario']['filename'])
        self.configurations_file = configurations_file
       
    def runCampaign(self,curCampaign):
        # Configure simulation file in accordance with campaign parameter
        sh_name = self.campaign_name + '_' + curCampaign
        print(curCampaign+" campaign written in file: " 'run_%s.sh' % sh_name)
        with open('run_%s.sh' % sh_name, 'w') as f:                    
            
            f.write('#!/bin/bash\n')
            if simLocation == 'cluster':
                f.write('#SBATCH --time=10-00:00          #Tempo máximo do job no formato DIAS-HORAS:MINUTOS\n')
                f.write('module load compilers/gnu/7.3\n')
                f.write('module load softwares/python/3.7-anaconda-5.0.1\n')
            
            outputDir = self.ns3_path+'/results_'+ self.filename + '_' + curCampaign
            f.write('rm -rf '+outputDir+' 2>/dev/null\n')
            f.write('mkdir -p '+outputDir+'\n')
            f.write('cp -f run_'+sh_name+'.sh'+' '+outputDir+'\n')
            f.write('cp -f '+self.configurations_file+ ' ' +outputDir+'\n')
            f.write("cd '"+self.ns3_path+"'"+"\n")
            ic = 1;
            for iJob in range(0, self.nJobs):
                jobRunSeed = random.randint(1, 23*10**14)
                for curLine in self.doc['scenario'][self.campaignLines]:
                    for varParam in self.doc['scenario'][curCampaign]:
                        command = (
                        'NS_GLOBAL_VALUE="RngRun='+str(jobRunSeed)+ '" ' +
                        "./waf --run '"+self.ns3_script+
                        " --radius="+self.radius+
                        " --nDevices="+self.nDevices+
                        " --simulationTime="+self.simulationTime+
                        " --appPeriodSeconds="+self.appPeriodSeconds+
                        " --print="+self.bPrint+
                        " --fixedSeed="+str(self.fixedSeed)+
                        " --algoritmo="+self.algoritmo+
                        " --filename="+ self.filename +
                        " --outputDir='"+outputDir+"'"
                        " --"+self.campaignLines+"="+curLine+
                        " --"+curCampaign+"="+varParam+
                        "'"
                        )
                        if ((ic % 6) == 0):
                            f.write(command+' & wait\n')
                            ic = 0;
                        else:
                            f.write(command+' & wait\n')
                        ic = ic + 1;    
                
        
                      
parser = argparse.ArgumentParser()
parser.add_argument("-f", "--file", type=str, help='Configuration File')
args = parser.parse_args()

configurations_file = args.file; 
with open(configurations_file, 'r') as f:
    doc = yaml.load(f, Loader=yaml.loader.BaseLoader)
    campaign_name = os.path.splitext(configurations_file)[0]

print('Simulação escolhida: ')
campaign = doc['scenario']['campaignX']
print(campaign)

simLocation = str(doc['scenario']['simLocation'])
if simLocation == 'cluster':
    with open('run_NPAD_%s.sh' % campaign_name, 'w') as fnpad:
        fnpad.write('#!/bin/bash\n')
        #fnpad.write('#SBATCH --time=10-00:00          #Tempo máximo do job no formato DIAS-HORAS:MINUTOS\n')
        
simu = Simulation(configurations_file)



for simC in campaign:
    if str(simC) == 'nDevices' or str(simC) == 'radius' or str(simC) == 'targetRealocation':
        simu.runCampaign(simC);
        if simLocation == 'cluster':
            with open('run_NPAD_%s.sh' % campaign_name, 'a') as fnpad:
                sh_name = campaign_name + '_' + simC
                fileName = ('run_%s.sh' % sh_name);
                fnpad.write('sbatch '+fileName+'\n')

    else:
        print('Invalid simulation campaign: verify the campaign parameter!')