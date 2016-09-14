#!/usr/bin/env python2
#Developed by Monroe Kennedy for CIS 580 Structure for motion University of Pennsylvania  March 2016
import os
import struct
import argparse
import sys
import glob
import sys, getopt
import glob
import scipy.io as sio

class Feature_struct(object):
    def __init__(self):
        self.match_names = []
        self.features_list = []


class Manipulate_txt_files(object):
    def generate_list_of_files(self):
        #reads all relavent files in this directory
        directory = os.listdir("SfMProjectData_1/")
        match_names = []
        for idx, fname in enumerate(directory):
            if 'matching' in fname:
                match_names.append(fname)
        try:
            match_names.remove('matching_feature_reader.py')
        except:
            pass
        self.match_names = match_names


    def read_file_in(self):
        #this function reads the file, then puts there inputs into function
        feature_list = []
        for file_names in self.match_names:
            img_num = file_names[-5]  #get the number of the current img
            fname = "SfMProjectData_1/" + file_names
            file = open(fname)
            file_mat = [list(file.read())]
            feat_struct = Feature_struct()
            feat_struct.match_names = file_names[:-4]
            feat_struct.features_list = file_mat
            feature_list.append(feat_struct)
        feature_list = list(feature_list)
        features_row_sorted = []
        for jdx in range(len(feature_list)):
            new_feat_struct = Feature_struct()
            l = feature_list[jdx].features_list[0]
            feature_rows = []
            #now find all the new lines: 
            new_lines = [i for i, x in enumerate(l) if x == "\n"]
            for idx in range(len(new_lines)-1):
                idx1 = new_lines[idx]
                idx2 = new_lines[idx+1]
                row = l[(idx1+1):idx2]
                spaces = [i for i, x in enumerate(row) if x ==" "]
                for index in range(len(spaces)-1):
                    row[spaces[index]] = ","
                feature_rows.append(row)
            new_feat_struct.match_names = feature_list[jdx].match_names
            new_feat_struct.features_list = feature_rows
            features_row_sorted.append(new_feat_struct)
        sio.savemat('feature_list.mat', {'feature_cell':features_row_sorted})


def main():
    #this file takes matching features and makes
    cls_obj = Manipulate_txt_files()
    cls_obj.generate_list_of_files()
    cls_obj.read_file_in()



if __name__ == "__main__":
    main()
