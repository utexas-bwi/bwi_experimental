import os,sys,operator,pickle,time
import parse_to_asp,dialogue_manager

path_to_main = sys.argv[1]

#path to SPF jar
path_to_spf = os.path.join(path_to_main,'..','spf','dist','spf-1.5.5.jar')
#path to write-able experiment directory
path_to_experiment = os.path.join(path_to_main,'..','spf','geoquery','experiments','template','dialog_writeable')		#For offline mode, this is the master writeable
path_to_master_dir = os.path.join(path_to_main,'..','spf','geoquery','experiments','template')
master_dir = 'dialog_writeable'
#path to ASP directory
path_to_asp = os.path.join(path_to_main,'..','asp')

def get_known_words_from_seed_files():

	seed_words = {}
	word_to_ontology_map = {}
	for filename in ['np-list.lex','seed.lex']:
		f = open(os.path.join(path_to_experiment,'resources',filename))
		for line in f:
			if (len(line) <= 1 or line[:2] == "//"):
				continue
			[token_sequence,tag_and_grounding] = line.split(" :- ")
			to_add = tag_and_grounding.strip().split(" : ")
			if (filename == 'np-list.lex'): #only add to seed words those words which are already grounded (ie, no CCG tags)
				seed_words[token_sequence] = to_add
			word_to_ontology_map[to_add[1].split(":")[0].strip()] =to_add[1].split(":")[1].strip()
	return seed_words,word_to_ontology_map
			
def retrain_parser_with_pairs(utterance_parse_pairs):

	if (len(utterance_parse_pairs) > 0):
		f = open(os.path.join(path_to_experiment,'data','fold0_train.ccg'),'w')
		f_init = open(os.path.join(path_to_experiment,'data','fold0_init_train.ccg'),'a')
		for (utterance,parse) in utterance_parse_pairs:
			f.write("\n"+utterance+"\n"+parse+"\n")
			f_init.write("\n"+utterance+"\n"+parse+"\n")
			print "added training pair \""+utterance+"\" -> "+parse
		f.close()
		f_init.close()
		print "augmenting system with new training data..."
		os.system('java -jar '+path_to_spf+' '+os.path.join(path_to_experiment,'train.exp'))
		print "done"

# def ask_for_grounding_confirmation(dm,grounding_instance_cutoff=3):
# 
# 	#find highest-confidence alignment guess and ask about it
# 	best_unmapped_guess = None
# 	best_unmapped_score = None
# 	changed_ontology = False
# 	for unmapped in dm.all_unmapped_words:
# 		candidate_score = max([dm.all_unmapped_words[unmapped][w] for w in dm.all_unmapped_words[unmapped]])
# 		if (best_unmapped_guess == None or best_unmapped_score < candidate_score):
# 			best_unmapped_guess = unmapped
# 			best_unmapped_score = candidate_score
# 	confirmed_not_aligned = []
# 	for (w,v) in sorted(dm.all_unmapped_words[best_unmapped_guess].iteritems(), key=operator.itemgetter(1), reverse=True):
# 		if (v < grounding_instance_cutoff):
# 			break
# 		dm.vocalize("Does '"+best_unmapped_guess+"' mean '"+w+"' ?")
# 		user_utterance_text = raw_input()
# 		if (user_utterance_text == "yes"):
# 			dm.vocalize("I thought so")
# 			f = open(os.path.join(path_to_experiment,'resources','np-list.lex'),'a')
# 			f.write(best_unmapped_guess+" :- "+given_words[w][0]+" : "+given_words[w][1]+"\n")
# 			given_words[best_unmapped_guess] = given_words[w] #can ground to this entry as well, in the future
# 			f.close()
# 			changed_ontology = True
# 			del dm.all_unmapped_words[best_unmapped_guess]
# 			break
# 		elif (user_utterance_text == "no"):
# 			dm.vocalize("I see")
# 			confirmed_not_aligned.append(w)
# 	if (best_unmapped_guess in dm.all_unmapped_words): #if we weren't able to find an alignment
# 		for w in confirmed_not_aligned: #remove words a user confirmed to be wrong
# 			del dm.all_unmapped_words[best_unmapped_guess][w]
# 		if (len(dm.all_unmapped_words[best_unmapped_guess]) == 0): #if we have no guesses anymore, don't keep the word around
# 			del dm.all_unmapped_words[best_unmapped_guess]
# 
# 	#if ontology was changed, we need to retrain the whole system
# 	return changed_ontology
	
def clingo_generate_plan_from_goal(goal_state):

	#write conditions file with goal state
	f = open(os.path.join(path_to_asp,'combined.asp'),'r')
	plan_asp_filename = os.path.join(path_to_asp,'temp.asp')
	f_out = open(os.path.join(path_to_asp,'temp.asp'),'w')
	f_out.write(f.read())
	f.close()
	f_out.write(":- not "+goal_state+".\n")
	f_out.close()
	
	#run clingo and read in resulting plan
	plan_output_filename = os.path.join(path_to_asp,'temp.plan')
	plan_found = "I couldn't figure out a way to execute that command."
	for n in range(0,50):
		os.system(os.path.join(path_to_asp,"clingo-4.3.0-source/build/release/clingo")+" "+plan_asp_filename+" -c n="+str(n)+" > "+plan_output_filename)
		f = open(plan_output_filename,'r')
		f_lines = f.read().split("\n")
		f.close()
		if ("Answer: 1" in f_lines): #satisfiable
			plan_found = f_lines[f_lines.index("Answer: 1")+1]
			break			
	return plan_found

#process arguments
run_offline = None
log_filename = None
restart_master_parser = False
retrain_master_parser = False
exclude_test_goals = False
test_goals = ['query(gave-up)','at(l3_512,n)','at(l3_416,n)','served(shiqi,phone,n)','served(shiqi,trashcan,n)','served(shiqi,coffee,n)','served(kazunori,calendar,n)','served(kazunori,hamburger,n)','served(stacy,calendar,n)','served(stacy,trashcan,n)','served(peter,calendar,n)','served(daniel,trashcan,n)','served(matteo,phone,n)']
if (len(sys.argv) >= 3 and sys.argv[2] == "retrain"):
		run_offline = True
		log_filename = False
		retrain_master_parser = True
elif (len(sys.argv) >= 4):
	if (sys.argv[2] == "online"):
		run_offline = False
		poll_filename = None
		push_filename = None
	elif (sys.argv[2] == "offline"):
		run_offline = True
	session_id = sys.argv[3]
	log_filename = os.path.join(path_to_main,"offline_data","logs",session_id+"_log.txt")
	alog_filename = os.path.join(path_to_main,"offline_data","alignments",session_id+"_alog.txt")
	poll_filename = os.path.join(path_to_main,"offline_data","inputs",session_id+"_input.txt")
	push_filename = os.path.join(path_to_main,"offline_data","outputs",session_id+"_output.txt")
	core_filename = os.path.join(path_to_main,"offline_data","cores",session_id+"_core.pickle")
	command_filename = os.path.join(path_to_main,"offline_data","commands",session_id+"_command.txt")
for i in range(2,len(sys.argv)):
	if (sys.argv[i] == "-restart_parser"):
		restart_master_parser = True
	elif (sys.argv[i] == "-exclude_test_goals"):
		exclude_test_goals = True
if (run_offline == None or log_filename == None):
	print "USE: $python main.py [path_to_main] [retrain/online/offline] session_id [-restart_parser] [-exclude_test_goals]"
	sys.exit()
	
#if excluding test goals, we need to read in the list of bad data IDs
bad_data_ids = []
if (exclude_test_goals == True):
	print "getting list of bad data IDs to exclude as well..."
	try:
		f = open(os.path.join(path_to_main,'list_of_bad_data.txt'),'r')
		for line in f:
			bad_data_ids.append(line[:-4])
		f.close()
	except:
		print "\tcouldn't find list_of_bad_data.txt"
	print bad_data_ids #DEBUG
	print "done"
	
#train parser
if (restart_master_parser == True):
	#train initial parser
	print "training master system with initial data..."
	os.system('java -jar '+path_to_spf+' '+os.path.join(path_to_experiment,'init_train.exp'))
	print "done"
	
#train parser with new alignments
if (retrain_master_parser == True):
	print "archiving previous parser, adding training data from last batch and augmenting parser, and archiving log data..."
	
	#archive previous master parser model
	retrain_time = str(time.time())
	os.system("cp -R "+path_to_experiment+" "+os.path.join(path_to_master_dir,master_dir+"_"+retrain_time))
	
	#trace through alignments files and add them to the training structure
	utterance_parse_pairs = []
	for root,dirs,files in os.walk(os.path.join(path_to_main,"offline_data","alignments")):
		for f in files:
			if (f.split('.')[1] == "txt"): #right filetype
			
				#maybe exclude user based on goal
				if (exclude_test_goals == True):
					user_id = "_".join(f.split("_")[:-1])
					if (user_id in bad_data_ids):
						print "DEBUG: skipped user '"+user_id+"' because they are marked as bad data"
						continue
					if (not os.path.exists(os.path.join(path_to_main,"offline_data","commands",user_id+"_command.txt"))):
						print "DEBUG: skipped user '"+user_id+"' because their dialog produced no command"
						continue
					g = open(os.path.join(path_to_main,"offline_data","commands",user_id+"_command.txt"),'r')
					command = g.read().strip()
					if (command in test_goals):
						print "DEBUG: skipped user '"+user_id+"' because their dialog led to test goal '"+command+"'"
						continue
			
				alog_f = open(os.path.join(root,f),'r')
				alog_f_lines = alog_f.read().strip().split("\n")
				if (len(alog_f_lines) < 2): #blank
					continue
				i = 0
				while (i < len(alog_f_lines)):
					utterance_parse_pairs.append((alog_f_lines[i],alog_f_lines[i+1]))
					i += 3
					if (i+1 >= len(alog_f_lines)):
						break
	
	#retrain parser
	retrain_parser_with_pairs(utterance_parse_pairs)
	
	#archive log data and empty active directories for next batch
	os.system("cp -R "+os.path.join(path_to_main,'offline_data')+" "+os.path.join(path_to_main,'offline_data_'+retrain_time))
	for dir in ['alignments','cores','inputs','logs','outputs','commands']:
		os.system("rm "+os.path.join(path_to_main,'offline_data',dir,'*'))
		
	print "done"
	sys.exit()

#get known word tokens from initialization files
given_words,word_to_ontology_map = get_known_words_from_seed_files()

#offline command function
if (run_offline == True):

	#instantiate dialogue manager
	print "creating dialogue manager..."
	session_master = session_id+"_"+master_dir
	if (os.path.exists(os.path.join(path_to_master_dir,session_master)) == False): #create per-user parser so log file conflicts don't happen
		os.system("cp -R "+path_to_experiment+" "+os.path.join(path_to_master_dir,session_master))
	dm = dialogue_manager.dialogue_manager(parse_to_asp.parse_to_asp(path_to_asp,word_to_ontology_map), os.path.join(path_to_master_dir,session_master), path_to_spf, given_words, log_filename, run_offline, alog_filename=alog_filename, poll_filename=poll_filename, push_filename=push_filename, core_filename=core_filename, session_id=session_id)

	asp_goal_state = dm.get_command_from_user_offline() #loads user utterance from given input file as response to DM state; might result in a goal state
	if (asp_goal_state == None):
		sys.exit("WARNING: reached main before shutting down")
	elif (asp_goal_state == False):
		print "WARNING: user gave up on achieving goal"
		asp_goal_state = "query(gave-up)" #ensures segbot dialog manager won't crash on no command
		f = open(command_filename,'w')
		f.write(asp_goal_state)
		f.close()
		os.system("rm -R "+os.path.join(path_to_master_dir,session_master)) #remove user-specific parsing model to save disk space
		dm.write_core_elements_to_pickle_and_shutdown() #write output vocalization(s) and save final core
	else:
		print asp_goal_state
		f = open(command_filename,'w')
		f.write(asp_goal_state)
		f.close()
		os.system("rm -R "+os.path.join(path_to_master_dir,session_master)) #remove user-specific parsing model to save disk space
		dm.write_core_elements_to_pickle_and_shutdown() #write output vocalization(s) and save final core

#online command loop
else:

	print "creating dialogue manager..."
	session_master = session_id+"_"+master_dir
	if (os.path.exists(os.path.join(path_to_master_dir,session_master)) == False): #create per-user parser so log file conflicts don't happen
		os.system("cp -R "+path_to_experiment+" "+os.path.join(path_to_master_dir,session_master))
	dm = dialogue_manager.dialogue_manager(parse_to_asp.parse_to_asp("../asp/",word_to_ontology_map), path_to_experiment, path_to_spf, given_words, log_filename, run_offline)

	ontology_changed = False
	while (True):

		#print "ROBOT STATE? [awaiting input, executing action, idle, shutdown]"
		print "ROBOT STATE? [awaiting input, idle, shutdown]"
		robot_state = raw_input()

		#if robot is awaiting user input
		if (robot_state == "awaiting input"):
	
			#get a verified parse of a command from the user
			asp_goal_state = dm.get_command_from_user()
			if (asp_goal_state == None):
				continue
			print "MAIN: goal returned: '"+asp_goal_state+"'"
			#simulate the execution of this action by showing the plan
			if (asp_goal_state.split("(")[0] != "query"):
				generated_plan = clingo_generate_plan_from_goal(asp_goal_state)
				print "MAIN: here's the plan of action which would be executed: '"+str(generated_plan)+"'"
			else:
				print "MAIN: no plan generated to answer query"
			#remove user-specific parsing model to save disk space
			os.system("rm -R "+os.path.join(path_to_master_dir,session_master))
			
		#if robot is performing a task
		# elif (robot_state == "executing action"):
# 	
# 			#request passive dialogue if there are unmapped words
# 			grounding_instance_cutoff = 3 #adjustable parameter; should be set somewhere else eventually
# 			if (len(dm.all_unmapped_words) == 0 or max([dm.all_unmapped_words[w1][w2] for w1 in dm.all_unmapped_words for w2 in dm.all_unmapped_words[w1]]) < grounding_instance_cutoff):
# 				continue
# 			dm.vocalize("Can I ask you some questions?")
# 			user_utterance_text = raw_input()
# 			if (user_utterance_text != "yes" and user_utterance_text != "sure"):
# 				dm.vocalize("That's fine")
# 				continue
# 			else:
# 				q = 0 #ask for clarifications on up to two words; adjustable parameter; should be set somewhere else eventually
# 				while (q < 2 and len(dm.all_unmapped_words) > 0):
# 					ontology_changed = ontology_changed or ask_for_grounding_confirmation(dm,grounding_instance_cutoff)
# 					q += 1
			
		#if robot is idle
		elif (robot_state == "idle"):
	
			#retrain parser and clear pairs
			retrain_parser_with_pairs(dm.utterance_parse_pairs)
			dm.utterance_parse_pairs = []
		
			#retrain whole system if ontology has been modified
			if (ontology_changed == True):
				print "re-training system with initial data plus known training examples plus new ontology..."
				os.system('java -jar '+path_to_spf+' '+os.path.join(path_to_experiment,'init_train.exp'))
				print "done"
				ontology_changed = False
				given_words,word_to_ontology_map = get_known_words_from_seed_files()
				dm.pta = parse_to_asp.parse_to_asp("../asp/",word_to_ontology_map)
				dm.new_given_words(given_words)
			
		#if robot is told to shut down
		elif (robot_state == "shutdown"):
	
			#pickle the dialogue manager, which carries word ontology information
			f = open("dm.pickle",'wb')
			pickle.dump(dm,f)
			f.close()
			sys.exit()
