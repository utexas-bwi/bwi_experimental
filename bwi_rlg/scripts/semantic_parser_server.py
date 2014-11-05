#!/usr/bin/env python

from bwi_rlg.srv import *
import rospy
import time
import subprocess
import os.path
import os
import glob

id_curr = ""

def get_id():

    return time.strftime("ID-%Y-%m-%d---") + str(time.time()).replace('.', '-')

def handle_semantic_parser(req):

    global id_curr

    if req.type == 0: # QUESTION_ASKING

        rospy.loginfo("Human: " + req.input_text)

        path_to_bwi_rlg = rospy.get_param(\
                "/semantic_parser_server/path_to_bwi_rlg")

        path_to_main = path_to_bwi_rlg + "/agent/dialog/"
        file_last_comm =  path_to_main + "last_comm_time.txt"

        time_curr = time.time()
        
        rospy.loginfo("file_last_comm: " + file_last_comm)

        if os.path.exists(file_last_comm):

            f = open(file_last_comm, 'r')
            time_last = f.readline().replace("\n", "")
            id_last = f.readline().replace("\n", "")
            f.close()
            # print("time_curr: " + str(time_curr))
            # print("time_last: " + str(time_last))
            time_diff = float(time_curr) - float(time_last)

        else:

            id_last = get_id() 
            time_diff = 0

        filelist = glob.glob(path_to_main + "offline_data/outputs/*")
        [os.remove(f) for f in filelist]


        if req.input_text.find("STARTING-KEYWORD") < 0 and \
                time_diff < rospy.get_param(\
                "/semantic_parser_server/patience_time_in_conversation"):

            f = open(path_to_main + "offline_data/inputs/" + id_last + \
                    "_input.txt", 'w+')
            f.write(req.input_text)
            f.close()
            
            cmd = "python " + path_to_main + "main.py " + path_to_main + \
                    "/ offline " + id_last
            subprocess.Popen(cmd, shell=True)

            output_file = path_to_main + "offline_data/outputs/" + id_last + \
                "_output.txt"

            while True:
                if os.path.exists(output_file):
                    diff = abs(time.time() - os.path.getmtime(output_file))
                    if diff < 1:
                        break
                else:
                    time.sleep(0.1)

            while os.path.exists(output_file) == False:
                time.sleep(0.1)

            f = open(output_file, 'r')
            output = f.read()
            f.close()

            f = open(file_last_comm, 'w+')
            f.write(str(time.time()) + '\n')
            f.write(id_last)
            f.close()

            id_curr = id_last

        else: 

            id_new = get_id()
            
            f = open(path_to_main + "offline_data/inputs/" + id_new + \
                    "_input.txt", 'w+')
            f.write(req.input_text)
            f.close()

            cmd = "python " + path_to_main + "main.py " + path_to_main + \
                    "/ offline " + id_new
            subprocess.Popen(cmd, shell=True)

            output_file = path_to_main + "offline_data/outputs/" + id_new + \
                "_output.txt"

            while os.path.exists(output_file) == False:
                time.sleep(0.1)

            f = open(output_file, 'r')
            output = f.read()
            f.close()

            f = open(file_last_comm, 'w+')
            f.write(str(time.time()) + '\n')
            f.write(id_new)
            f.close()
           
            id_curr = id_new

        path_to_command = path_to_main + "offline_data/commands/" + id_curr + \
                          "_command.txt"
        if os.path.exists(path_to_command):
            f = open(path_to_command, 'r')
            command = f.readline()
            command = command.replace("\n", "")
            rospy.loginfo("path_to_command: " + command)
        else:
            command = ""
            

        return SemanticParserResponse(str(output), command)

    elif req.type == 1: # TRAINING
        
        res = "Training: not implemented."
        rospy.logwarn(res)
        return SemanticParserResponse(res)

    elif req.type == 2: # STARTOVER

        res = "Start over: not implemnted."
        rospy.logwarn(res)
        return SemanticParserResponse(res)

    elif req.type == 3: # GETID

        return SemanticParserResponse(id_curr, "")

    else:
        rospy.logerr("Error in semantic_parser_server.")

def semantic_parser_server():

    rospy.init_node('semantic_parser_server')
    s = rospy.Service('semantic_parser', SemanticParser, handle_semantic_parser)
    rospy.spin()


if __name__ == "__main__":

    semantic_parser_server()   

