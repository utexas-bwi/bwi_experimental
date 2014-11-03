import os,sys,random,time,pickle,string

#import pyaudio
#import wave

class dialogue_manager:

	def __init__(self, pta, path_to_experiment, path_to_spf, given_words, log_filename, run_offline, alog_filename=None, poll_filename=None, push_filename=None, core_filename=None, session_id=None):
	
		#input types
		self.vocalize_with_TTS = False
		self.input_with_ASR = False
		
		#manager tools
		self.pta = pta
		self.path_to_experiment = path_to_experiment
		self.path_to_spf = path_to_spf
		self.known_words = given_words
		self.new_given_words(given_words)
		self.log_filename = log_filename
		self.run_offline = run_offline #if running offline, operations happen step-wise instead of continuously between user inputs (ie system shuts down)
		self.alog_filename = alog_filename
		self.poll_filename = poll_filename
		self.push_filename = push_filename
		self.core_filename = core_filename
		self.session_id = session_id
		self.responses_from_robot = []
		
		#manager state information
		self.current_best_asp_understanding = [[None,0],[None,0],[None,0]] #action, patient, recipient triple and their confidences
		self.current_asp_confidence = [{},{},{}] #distributions of confidence over possible grounded referents
		self.asp_role_map = {"action":0,"patient":1,"recipient":2}
		self.request_type = "user_initiative" #can also be system-initiative,terminate
		self.role_requested = None #if system-initiative request, this holds the role requested from the user
		self.last_apr = None
		self.last_role_requested = None #these two together form the previous state chosen by the dialogue manager
		#self.all_unmapped_words = {} #to be used during clarification dialogues and for retraining ontology
		self.utterance_parse_pairs = [] #to be used during retraining
		self.roles_relatively_confident_about = None
		
		#local state information for the dialogue
		self.utterances_for_goal = [] #local utterances used during current dialogue to describe high-level, open-ended goal (cleared each time new dialogue starts)
		self.utterances_for_clarification = [] #used in alignment guesses but need to be kept separate from main goal utterances and other clarification sub-dialogue utterances
		self.utterances_during_dialogue = {} #indexed by request type (goal, sentence component), contains lists of utterances used to describe same goal/component for re-training
		self.dialogue_accepted_parses = {} #indexed by request type (goal, sentence component), contains parse chosen as correct for that request
		#self.unmapped_words_from_utterances = [] #unmapped words from utterances in current dialogue (wrapped into all unmapped words structure at dialogue's end)
		
		#manager classification parameters
		self.utterance_class_keywords = {}
		self.utterance_class_keywords["yes"] = ["yes","yeah","sure","right","correct"]
		self.utterance_class_keywords["no"] = ["no","nope","incorrect","wrong","not right"]
		self.utterance_class_keywords["nevermind"] = ["nevermind","wrong","misunderstood","don't","dont","nothing"]
		
		#manager adjustable / learnable parameters
		self.max_asr_understandings_to_consider = 4
		self.min_confidence_to_accept = 0.95 #when all arguments of understanding meet this confidence, dialogue terminates
		self.confidence_decay_rate = 0.5 #every time the user gives a free response for a command and a past argument is not mentioned again, decay its confidence by this rate
	
	#write core elements to a pickle file indexed by session ID, then shut down
	def write_core_elements_to_pickle_and_shutdown(self):
	
		#write core
		f = open(self.core_filename,'wb')
		pickle.dump([self.current_asp_confidence,self.request_type,self.role_requested,self.last_apr,self.last_role_requested,self.roles_relatively_confident_about,self.current_best_asp_understanding,self.utterances_for_goal,self.utterances_for_clarification,self.utterances_during_dialogue,self.dialogue_accepted_parses],f)
		f.close()
		
		#shutdown
		f = open(self.push_filename,'w')
		f.write("\n".join(self.responses_from_robot)+"\n")
		f.close()
		self.responses_from_robot = []
		sys.exit()
		
	#load core elements from a pickle file indexed by session ID
	def load_core_elements_from_pickle(self):
		f = open(self.core_filename,'rb')
		[self.current_asp_confidence,self.request_type,self.role_requested,self.last_apr,self.last_role_requested,self.roles_relatively_confident_about,self.current_best_asp_understanding,self.utterances_for_goal,self.utterances_for_clarification,self.utterances_during_dialogue,self.dialogue_accepted_parses] = pickle.load(f)
		f.close()
	
	#load a mapping from grounded terms to lexicon referring expressions from given word map
	def new_given_words(self, given_words):
	
		self.grounded_to_lexicon_map = {}
		for w in given_words:
			key = given_words[w][1].strip()
			if (key in self.grounded_to_lexicon_map):
				self.grounded_to_lexicon_map[key].append(w)
			else:
				self.grounded_to_lexicon_map[key] = [w]
	
	#some boilerplate; max_argmax function
	def max_argmax(self,d):
		m = None
		for key in d:
			if (m == None):
				m = key
			if (d[key] > d[m]):
				m = key
		if (m == None):
			return 0, None
		else:
			return d[m], m
	
	#boilerplate; dict add function adds all elements of dict B to dict A
	def dict_add(self,A,B):
		for key in B:
			if (type(B[key]) is list):
				if (key in A):
					A[key].extend(B[key])
				else:
					A[key] = B[key][:]
			elif (type(B[key]) is int):
				if (key in A):
					A[key] += B[key]
				else:
					A[key] = B[key]
			else:
				print "dict_add:\tunsupported element type '"+str(type(B[key]))+"'"
	
	#show text and/or render speech to the user
	def vocalize(self, response, text=None, request_type=None, role_requested=None):
		
		if (text == None):
			text = not self.vocalize_with_TTS
		if (text == True):
			print "ROBOT: "+response
		else:
			print "ROBOT: "+response
			os.system("/usr/share/speak \""+response+"\"")
			
		if (request_type == None):
			request_type = self.request_type
		if (role_requested == None):
			role_requested = self.role_requested
			
		self.responses_from_robot.append(response)
		
		f = open(self.log_filename,'a')
		f.write("\t".join(["ROBOT",response,str(self.current_asp_confidence),str(self.roles_relatively_confident_about),str(request_type),str(role_requested)])+"\n")
		f.close()
	
	#choose a referring expression given a grounded term
	def choose_referring_expression(self, gw, words_to_avoid=''):
	
		#don't throw errors if we get bad input; just send it back since it's not going to be verbalized anyway
		if (gw == None or gw == False):
			return None
			
		#if the gw is a digit, we just return that
		if (gw.isdigit() == True):
			return gw
			
		#select the first referring expression not contained in the string of words to avoid
		chosen_expression = None
		for i in range(0,len(self.grounded_to_lexicon_map[gw])):
			if (self.grounded_to_lexicon_map[gw][i] not in words_to_avoid):
				chosen_expression = self.grounded_to_lexicon_map[gw][i]
				
		#if the vocalization chosen is "me" or "i" from the ontology, we need to flip the POV for the pronoun
		if (chosen_expression == "me" or chosen_expression == "i"):
			chosen_expression = "you"
		
		#if we arrive here, then all the referring expressions for grounded term are in the words to avoid, so just take the first
		if (chosen_expression == None):
			chosen_expression = self.grounded_to_lexicon_map[gw][0]
		
		return chosen_expression
	
	#get text and/or speech from the user
	def get_user_input(self, text=None):
	
		#if offline, just read from poll file
		if (self.run_offline == True):
			#extract and sanitize
			f = open(self.poll_filename)
			user_input = f.read().strip().lower()
			user_input = user_input.replace("'s"," s")
			user_input = user_input.translate(string.maketrans("",""), string.punctuation)
			#log
			f = open(self.log_filename,'a')
			f.write("\t".join(["USER",user_input])+"\n")
			f.close()
			return [[user_input,0]] #full confidence value (log-probability) returned with text
			
		#else, do normal online processing
	
		if (text == None):
			text = not self.input_with_ASR
		if (text == True):
			user_input = raw_input()
			f = open(self.log_filename,'a')
			f.write("\t".join(["USER",user_input])+"\n")
			f.close()
			return [[user_input,0]] #full confidence value (log-probability) returned with text
			
		else:
		
			#TODO: set recording constants elsewhere
			CHUNK = 1024
			FORMAT = pyaudio.paInt16
			CHANNELS = 1
			RATE = 16000
			RECORD_SECONDS = 4
			
			#record user command
			fn = "user_utterance.wav"
			self.vocalize("Press 'enter' and then speak your command")
			_ = raw_input()
			p = pyaudio.PyAudio()
			stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
			print("DEBUG: *recording*")
			frames = []
			for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
				data = stream.read(CHUNK)
				frames.append(data)
			print("DEBUG: *done recording*")
			stream.stop_stream()
			stream.close()
			p.terminate()
			wf = wave.open(fn, 'wb')
			wf.setnchannels(CHANNELS)
			wf.setsampwidth(p.get_sample_size(FORMAT))
			wf.setframerate(RATE)
			wf.writeframes(b''.join(frames))
			wf.close()
			wavfile = fn
			
			#get n-best understanding from pocketsphinx
			results = []
			os.system("./sphinx_nbest 1> sphinx_nbest_stdout.txt 2> sphinx_nbest_stderr.txt")
			f = open("sphinx_nbest_stdout.txt",'r')
			for line in f:
				hyp,score = line.strip().split('\t')
				score = int(score)
				if (hyp not in [r[0] for r in results]):
					results.append([hyp,score])
				if (len(results) == self.max_asr_understandings_to_consider):
					break
			f.close()
			
			print "DEBUG: ASR results = "+str(results)
			return results
	
	#guess whether an utterance translates to "yes", "no", "nevermind", or something else
	def classify_user_utterances(self, utterances_text):
	
		#utterance_class_scores = {utterance_class:0 for utterance_class in self.utterance_class_keywords}
		utterance_class_scores = {}
		for utterance_class in self.utterance_class_keywords:
			utterance_class_scores[utterance_class] = 0
		for utterance_text in utterances_text:
			
			for utterance_class in self.utterance_class_keywords:
				for keyword in self.utterance_class_keywords[utterance_class]:
					if (keyword in utterance_text):
						#ad-hoc check for kazunori's name; definitely needs to be removed eventually
						if (keyword == 'no' and 'kazunori' in utterance_text):
							continue
						utterance_class_scores[utterance_class] += 1
					
		maximum_score,max_utterance_class = self.max_argmax(utterance_class_scores)
		print "utterance_class_scores="+str(utterance_class_scores)+"; max_utterance_class="+max_utterance_class #DEBUG
		if (maximum_score > 0):
			return max_utterance_class
		else:
			return None
	
	#invoke SPF parser and get the semantic parse(s) of the sentence, as well as any new unmapped words in the utterance
	def parse_utterance(self, user_utterance_text):

		f = open(os.path.join(self.path_to_experiment,'data','test.ccg'),'w')
		f.write(user_utterance_text+"\n(lambda $0:e $0)\n")
		f.close()

		#run parser and read output
		os.system('java -jar '+self.path_to_spf+' '+os.path.join(self.path_to_experiment,'test.exp'))
		f = open(os.path.join(self.path_to_experiment,'logs','load_and_test.log'),'r')
		lines = f.read().split("\n")
		parses = []
		current_unmapped_sequence = None #[sequence, last_index]
		unmapped_words_in_utterance = {}
		for i in range(0,len(lines)):
			if (' WRONG: ' in lines[i] or 'too many parses' in lines[i]): #found parses
				if (' WRONG: ' in lines[i] and len(lines[i].split('WRONG: ')[1]) > 0 and 'parses' not in lines[i].split('WRONG: ')[1]): #one parse
					parses.append((lines[i].split('WRONG: ')[1],0))
				else: #multiple parses
					j = 1 if ' WRONG: ' in lines[i] else 2
					while (' Had correct parses: ' not in lines[i+j]):
						if ('[S' not in lines[i+j]):
							p = lines[i+j][lines[i+j].index('[')+2:]
						else:
							p = lines[i+j].split(']')[2][1:]
						s = float(lines[i+j+1].split()[3])
						print s #DEBUG
						parses.append((p,s))
						j += 3
			elif ('EMPTY' in lines[i] and len(lines[i].split()) >= 4 and lines[i].split()[3] == "EMPTY"): #found unmapped word
				empty_token = lines[i].split()[1]
				if (current_unmapped_sequence == None):
					current_unmapped_sequence = [empty_token,i]
				elif (i-1 == current_unmapped_sequence[1]):
					current_unmapped_sequence[0] += " "+empty_token
					current_unmapped_sequence[1] = i
				else:
					if (current_unmapped_sequence[0] not in self.known_words):
						unmapped_words_in_utterance[current_unmapped_sequence[0]] = {}
					current_unmapped_sequence = [empty_token,i]
		if (current_unmapped_sequence != None and current_unmapped_sequence[0] not in self.known_words):
			unmapped_words_in_utterance[current_unmapped_sequence[0]] = {}
		f.close()

		return parses,unmapped_words_in_utterance
	
	#using list of previous utterances and unmapped words in a current utterance, make fresh alignment guesses
	# def update_unmapped_words_alignment_guesses(self, unmapped_words_from_utterances, utterances, unmapped_words_in_utterance):
# 	
# 		sys.stderr.write("DEBUG: update_unmapped_words_alignment_guesses called with unmapped_words_in_utterance="+str(unmapped_words_in_utterance)+"\n")
# 		unmapped_words_from_utterances.append(unmapped_words_in_utterance)
# 		user_utterance_text = utterances[-1]
# 		if (len(utterances) > 1):
# 			for prev in range(2,len(utterances)+1):
# 				novel_words_in_utterance = [w for w in user_utterance_text.split() if w not in utterances[-prev].split()]
# 				grounded_terms_in_utterance = []
# 				for i in range(0,len(novel_words_in_utterance)):
# 					for j in range(i+1,len(novel_words_in_utterance)+1):
# 						candidate = " ".join(novel_words_in_utterance[i:j])
# 						if " ".join(novel_words_in_utterance[i:j]) in self.known_words:
# 							grounded_terms_in_utterance.append(candidate)
# 				for unmapped_w in unmapped_words_from_utterances[-prev]:
# 					for w in grounded_terms_in_utterance:
# 						if (w in unmapped_words_from_utterances[-prev][unmapped_w]):
# 							unmapped_words_from_utterances[-prev][unmapped_w][w] += 1
# 						else:
# 							unmapped_words_from_utterances[-prev][unmapped_w][w] = 1
# 		sys.stderr.write("DEBUG: update_unmapped_words_alignment_guesses updated unmapped_words_from_utterances="+str(unmapped_words_from_utterances)+"\n")
	
	#using the mapping of alignment guesses for this set of utterances, update global guesses about referring expression meanings
	# def update_all_unmapped_words_guesses(self, unmapped_words_from_utterances):
# 	
# 		sys.stderr.write("DEBUG: update_all_unmapped_words_guesses called with current unmapped word/utterance structure "+str(unmapped_words_from_utterances)+"\n")
# 		for i in range(0,len(unmapped_words_from_utterances)):
# 			if (len(unmapped_words_from_utterances[i]) > 0):
# 				for unmapped_w in unmapped_words_from_utterances[i]:
# 					if (len(unmapped_words_from_utterances[i][unmapped_w]) > 0):
# 						if (unmapped_w not in self.all_unmapped_words):
# 							sys.stderr.write("DEBUG: all_unmapped_words adding new count '"+unmapped_w+"'->'"+str(unmapped_words_from_utterances[i][unmapped_w])+"'\n")
# 							self.all_unmapped_words[unmapped_w] = unmapped_words_from_utterances[i][unmapped_w]
# 						else:
# 							for w in unmapped_words_from_utterances[i][unmapped_w]:
# 								sys.stderr.write("DEBUG: all_unmapped_words adding to existing count '"+unmapped_w+"'->'"+w+"'\n")
# 								if (w in self.all_unmapped_words[unmapped_w]):
# 									self.all_unmapped_words[unmapped_w][w] += unmapped_words_from_utterances[i][unmapped_w][w]
# 								else:
# 									self.all_unmapped_words[unmapped_w][w] = unmapped_words_from_utterances[i][unmapped_w][w]

	#get the action, patient, and recipient from an asp_node based on dialogue knowledge of possible actions
	#for ambiguous parses (lists of nodes), distribute confidence based on the number of candidate trees in which various candidate groundings exist
	def get_apr_tuple_from_asp_node(self, node):
		if (type(node) is not list):
			print "DEBUG: get_apr_tuple_from_asp_node: called on node="+str(node)+" with function "+str(node.function)+" and args "+str(node.arguments) #DEBUG
			if (node.function == "served"):
				return [{"served":self.min_confidence_to_accept},{node.arguments[1]:self.min_confidence_to_accept if node.arguments[1] != None else 0},{node.arguments[0]:self.min_confidence_to_accept if node.arguments[0] != None else 0}]
			elif (node.function == "at"):
				return [{"at":self.min_confidence_to_accept},{False:self.min_confidence_to_accept},{node.arguments[0]:self.min_confidence_to_accept if node.arguments[0] != None else 0}]
			elif (node.function == "query"):
				return [{"query":self.min_confidence_to_accept},{node.arguments[0]:self.min_confidence_to_accept if node.arguments[0] != None else 0},{False:self.min_confidence_to_accept}]
			else:
				sys.stderr.write("ERROR: unrecognized asp node root function '"+node.function+"' in dialogue manager")
		else:
			print "DEBUG: get_apr_tuple_from_asp_node: called on list of nodes="+str(node) #DEBUG
			apr_score_totals = [{},{},{}]
			for n in node:
				apr = self.get_apr_tuple_from_asp_node(n)
				for role in self.asp_role_map:
					if (apr[self.asp_role_map[role]].keys()[0] in apr_score_totals[self.asp_role_map[role]]):
						apr_score_totals[self.asp_role_map[role]][apr[self.asp_role_map[role]].keys()[0]] += apr[self.asp_role_map[role]][apr[self.asp_role_map[role]].keys()[0]]
					else:
						apr_score_totals[self.asp_role_map[role]][apr[self.asp_role_map[role]].keys()[0]] = apr[self.asp_role_map[role]][apr[self.asp_role_map[role]].keys()[0]]
			for role in self.asp_role_map:
				role_score_total = float(sum([apr_score_totals[self.asp_role_map[role]][w] for w in apr_score_totals[self.asp_role_map[role]]]))
				if (role_score_total > 0):
					for w in apr_score_totals[self.asp_role_map[role]]:
						apr_score_totals[self.asp_role_map[role]][w] = apr_score_totals[self.asp_role_map[role]][w] / role_score_total
			return apr_score_totals

	#given an apr tuple, generate natural language to describe it
	def verbalize_apr_tuple(self, apr):
	
		#generate referring expressions for arguments; if user is asking a question, don't use any of the words they used (it looks snarky)
		words_to_avoid = self.utterances_for_goal[-1] if apr[self.asp_role_map["action"]] == "query" else ''
		patient_ref = self.choose_referring_expression(apr[self.asp_role_map["patient"]],words_to_avoid)
		recipient_ref = self.choose_referring_expression(apr[self.asp_role_map["recipient"]],words_to_avoid)
		
		#generate full verbalization
		if (apr[self.asp_role_map["action"]] == "served"):
			return "bring "+patient_ref+" to "+recipient_ref
		elif (apr[self.asp_role_map["action"]] == "at"):
			return "walk to "+recipient_ref
		elif (apr[self.asp_role_map["action"]] == "query"):
			return patient_ref
		else:
			sys.stderr.write("ERROR: unrecognized apr action '"+apr[self.asp_role_map["action"]]+"' in dialogue manager")

	#get a response from user and return the text(s) and classification(s) of the response
	def get_user_response(self):
	
		#get and classify utterance
		user_utterances = self.get_user_input()
		user_utterance_class = self.classify_user_utterances([u[0] for u in user_utterances])
		
		return user_utterances,user_utterance_class
	
	#process unrestricted response from the user and update internal understanding confidences based on parse results	
	def process_user_initiative_response(self, user_utterances, user_utterance_class):
	
		#give up if user asked to
		if (user_utterance_class == "nevermind"):
			self.request_type = "terminate" #cease attempt to understand command
			self.role_requested = None
			return None
			
		#make a pass at understanding
		parses_in_utterance_understandings = []
		unmapped_words_in_utterance_understandings = []
		utterance_scores = []
		for user_utterance,utterance_score in user_utterances:
			utterance_scores.append(utterance_score)
			utterance_parses,unmapped_words_in_utterance_understanding = self.parse_utterance(user_utterance)
			parses_in_utterance_understandings.append(utterance_parses)
			unmapped_words_in_utterance_understandings.append(unmapped_words_in_utterance_understanding)
		
		#try to map each candidate parse into an ASP node
		asp_nodes = []
		highest_confidence_index = (0,None) #default to highest-confidence ASR for utterance/alignment
		highest_confidence_score = -sys.maxint
		for i in range(0,len(user_utterances)):
			parses = parses_in_utterance_understandings[i]
			sys.stderr.write("parses for understanding '"+user_utterances[i][0]+"'\n")
			sys.stderr.write(str(parses)+"\n")
			for j in range(0,len(parses)):
				asp_node = self.pta.answer_parse_with_asp(parses[j][0])
				if (asp_node != None and parses[j][1]+utterance_scores[i] > highest_confidence_score): #score is parse score plus ASR score to determine most confident parse
					highest_confidence_index = (i,j)
				if (type(asp_node) is list): #ambiguity; we will reduce our confidence in the extractions accordingly
					sys.stderr.write("DEBUG: asp translation yielded multiped valid instantiations for parse '"+parses[j][0]+"'\n")
					asp_nodes.append([i,j,asp_node])
				else:
					if (asp_node == None):
						sys.stderr.write("DEBUG: asp translation failed for parse '"+parses[j][0]+"'\n")
					else:
						sys.stderr.write("DEBUG: asp translation successful for parse '"+parses[j][0]+"'\n")
						asp_nodes.append([i,j,asp_node])
					
		#choose the "gold" user utterance for future parse (re)training and word alignment guessing
		print "DEBUG: highest confidence utterance: '"+str(user_utterances[highest_confidence_index[0]][0])+"'"
		if (user_utterances[highest_confidence_index[0]][0] == "(null)"): #the Sphinx ASR didn't pick up any words, so don't consider this further, even for retraining/alignment
			return None
		if (highest_confidence_index[1] != None): #else, there will have been no successful translations so we will return None when that check is made
			highest_confidence_parse = parses_in_utterance_understandings[highest_confidence_index[0]][highest_confidence_index[1]][0]
		user_utterance = user_utterances[highest_confidence_index[0]][0]
		unmapped_words_in_utterance = unmapped_words_in_utterance_understandings[highest_confidence_index[0]]
					
		#note utterance as a novel way of conveying user goal for this dialogue, then update alignment guesses
		self.utterances_for_goal.append(user_utterance)
		if ("goal" in self.utterances_during_dialogue):
			self.utterances_during_dialogue["goal"].append(user_utterance)
		else:
			self.utterances_during_dialogue["goal"] = [user_utterance]
		#self.update_unmapped_words_alignment_guesses(self.unmapped_words_from_utterances, self.utterances_for_goal, unmapped_words_in_utterance)
		
		#no asp translation produced even a partial ASP node
		if (len(asp_nodes) == 0):
			return None
			
		#update our confidence in various possible answers using an interpolation of parse confidence and asp ambiguity from multiple instantiation
		#role_candidates = {role:[] for role in self.asp_role_map}
		role_candidates = {}
		for role in self.asp_role_map:
			role_candidates[role] = []
		for i in range(0,len(asp_nodes)):
			apr_tuple = self.get_apr_tuple_from_asp_node(asp_nodes[i][2])
			for role in self.asp_role_map:
				for candidate in apr_tuple[self.asp_role_map[role]]:
					if (candidate == None):
						continue
					confidence = apr_tuple[self.asp_role_map[role]][candidate] / float(len(asp_nodes)) #confidence is asp ambiguity divided by parser ambiguity; (TODO: tie to parse confidence)
					role_candidates[role].append(candidate)
					if (candidate in self.current_asp_confidence[self.asp_role_map[role]]):
						self.current_asp_confidence[self.asp_role_map[role]][candidate] += (1-self.current_asp_confidence[self.asp_role_map[role]][candidate])*confidence
					else:
						self.current_asp_confidence[self.asp_role_map[role]][candidate] = confidence
						
		#decay confidences of things unmentioned in the newest parses
		for role in self.asp_role_map:
			for existing_candidates in self.current_asp_confidence[self.asp_role_map[role]]:
				if (existing_candidates not in role_candidates[role]):
					self.current_asp_confidence[self.asp_role_map[role]][existing_candidates] = self.current_asp_confidence[self.asp_role_map[role]][existing_candidates]*self.confidence_decay_rate
					
		return highest_confidence_parse

	#given a partial instantiation and a particular role to question about, form a coherent question
	def verbalize_query_from_partial_apr_tuple(self,apr,role_requested):
	
		sys.stderr.write("DEBUG: verbalize_query_from_partial_apr_tuple apr,role_requested: "+str(apr)+","+str(role_requested)+"\n")
		
		#if asked to verbalize partial, check whether last verbalization was identical and apologize for needing additional clarification
		if (self.last_apr != None and apr[self.asp_role_map["action"]] == self.last_apr[self.asp_role_map["action"]] and apr[self.asp_role_map["patient"]] == self.last_apr[self.asp_role_map["patient"]] and apr[self.asp_role_map["recipient"]] == self.last_apr[self.asp_role_map["recipient"]] and role_requested == self.last_role_requested):
			self.vocalize("I'm sorry, but I couldn't pinpoint what you meant by that.",request_type="no_clarification_gained",role_requested=None)
		self.last_apr = apr
		self.last_role_requested = role_requested
		
		#generate referring expressions for arguments
		words_to_avoid = self.utterances_for_goal[-1] if apr[self.asp_role_map["action"]] == "query" else ''
		patient_ref = self.choose_referring_expression(apr[self.asp_role_map["patient"]],words_to_avoid)
		recipient_ref = self.choose_referring_expression(apr[self.asp_role_map["recipient"]],words_to_avoid)
		
		if (role_requested == "action"):
			if (apr[self.asp_role_map["action"]] == None):
				if (patient_ref == None and recipient_ref != None):
					return "What action did you want me to take involving "+recipient_ref+"?"
				elif (patient_ref != None and recipient_ref == None):
					return "What action did you want me to take involving "+patient_ref+"?"
				else:
					return "What did you want me to do with "+patient_ref+" for "+recipient_ref+"?"
			elif (apr[self.asp_role_map["action"]] == "served"):
				if (patient_ref == None and recipient_ref != None):
					return "Should I deliver something to "+recipient_ref+"?"
				elif (patient_ref != None and recipient_ref == None):
					return "Should I bring "+patient_ref+" to someone?"
				else:
					return "I should deliver something?"
			elif (apr[self.asp_role_map["action"]] == "at"):
				return "I should walk somewhere?"
		
		elif (role_requested == "patient"):
			if (patient_ref == None):
				if (apr[self.asp_role_map["action"]] == None and recipient_ref != None):
					return "What will be received by "+recipient_ref+"?"
				elif (apr[self.asp_role_map["action"]] == "served" and recipient_ref == None):
					return "What should I bring?"
				elif (apr[self.asp_role_map["action"]] == "served" and recipient_ref != None):
					return "What should I bring to "+recipient_ref+"?"
			else:
				if (apr[self.asp_role_map["action"]] == None and recipient_ref != None):
					return "So "+patient_ref+" is for "+recipient_ref+"?"
				elif (apr[self.asp_role_map["action"]] == "served" and recipient_ref == None):
					return "So I am to deliver "+patient_ref+" to someone?"
					
		elif (role_requested == "recipient"):
			if (recipient_ref == None):
				if (apr[self.asp_role_map["action"]] == None and patient_ref != None):
					return "Who or what is "+patient_ref+" for?"
				elif (apr[self.asp_role_map["action"]] == "served"):
					if (patient_ref == None):
						return "To whom should I bring something?"
					else:
						return "To whom should I bring "+patient_ref+"?"
				elif (apr[self.asp_role_map["action"]] == "at"):
					return "Where should I walk?"
			else:
				if (apr[self.asp_role_map["action"]] == None):
					if (patient_ref != None):
						return "So "+patient_ref+" is for "+recipient_ref+"?"
					else:
						return "I should do something involving "+recipient_ref+"?"
				elif (apr[self.asp_role_map["action"]] == "served" and patient_ref == None):
					return "I am to deliver something to "+recipient_ref+"?"

	#using the current understanding confidence scores, determine the next state and verbalize a query for the user
	#this is, effectively, our policy function
	def articulate_next_state(self):
	
		#use confidence scores to determine what to ask about
		
		#let confidence influence which arguments we're going to consider relatively correct for this question
		relatively_confident_about = [None,None,None]
		confident_about_at_least_one = False
		for role in self.asp_role_map:
			if (random.random() < self.current_best_asp_understanding[self.asp_role_map[role]][1]):
				relatively_confident_about[self.asp_role_map[role]] = self.current_best_asp_understanding[self.asp_role_map[role]][0]
				if (relatively_confident_about[self.asp_role_map[role]] not in [False,None]):
					confident_about_at_least_one = True
		if (confident_about_at_least_one == False):
			verbal_query = "Sorry I couldn't understand that. Could you reword your original request?"
			self.request_type = "user_initiative"
			self.role_requested = None
			return verbal_query,relatively_confident_about
		else:
			self.request_type = "system_initiative"
		print "DEBUG: articulate_next_state relative confidence="+str(relatively_confident_about) #DEBUG
				
		#if unconfident about one or more arguments, ask for clarification of the one with the least confidence
		if (relatively_confident_about[self.asp_role_map["action"]] == "at"): #set single-argument constraints
			relatively_confident_about[self.asp_role_map["patient"]] = False
			self.role_requested = "recipient"
			if (relatively_confident_about[self.asp_role_map["recipient"]] == False): #if believing in walking but have chosen absent recipient, choose unknown instead
				relatively_confident_about[self.asp_role_map["recipient"]] = None
		if (relatively_confident_about[self.asp_role_map["action"]] == "query"):
			if (relatively_confident_about[self.asp_role_map["patient"]] == False):
				relatively_confident_about[self.asp_role_map["patient"]] = None
		if (relatively_confident_about[self.asp_role_map["action"]] == "served"): #set two-argument constraints
			if (relatively_confident_about[self.asp_role_map["recipient"]] == False):
				relatively_confident_about[self.asp_role_map["recipient"]] = None
			if (relatively_confident_about[self.asp_role_map["patient"]] == False):
				relatively_confident_about[self.asp_role_map["patient"]] = None
		print "DEBUG: articulate_next_state relative confidence after constraints="+str(relatively_confident_about) #DEBUG
		if (None in relatively_confident_about):
			if (relatively_confident_about[self.asp_role_map["action"]] == "query"): #can't clarify an unknown query argument
				verbal_query = "Sorry I couldn't understand that. Could you reword your question?"
				self.request_type = "user_initiative"
				self.role_requested = None
				return verbal_query,relatively_confident_about
			min_conf = min([self.current_best_asp_understanding[self.asp_role_map[role]][1] for role in self.asp_role_map])
			self.role_requested = [role for role in self.asp_role_map if self.current_best_asp_understanding[self.asp_role_map[role]][1] == min_conf][0]
			if (relatively_confident_about[self.asp_role_map["action"]] == "at"): #another set of constraints
				if (self.role_requested == "patient"): #if believe in walking, don't ask about known-to-be-False-patient
					self.role_requested = "recipient"
			print "articulate_next_state: current_best_asp_understanding="+str(self.current_best_asp_understanding)+"; min_conf="+str(min_conf)+"; role_requested="+str(self.role_requested) #DEBUG
			if (relatively_confident_about[self.asp_role_map[self.role_requested]] == None and "goal" in self.dialogue_accepted_parses):
				del self.dialogue_accepted_parses["goal"] #if we have to ask for clarification, then the global goal parse currently obtained can't be trusted
			verbal_query = self.verbalize_query_from_partial_apr_tuple(relatively_confident_about,self.role_requested)
		#if relatively confident about all arguments, ask for global confirmation
		else:
			if (relatively_confident_about[self.asp_role_map["action"]] == "query"):
				if (self.role_requested == "action"):
					self.role_requested = "patient"
					#verbal_query = "Are you asking a question about '"+self.verbalize_apr_tuple(relatively_confident_about)+"'?"
					verbal_query = "'"+self.verbalize_apr_tuple(relatively_confident_about)+"', does that answer your question?"
				else:
					self.role_requested = "action"
					verbal_query = "Are you asking me a question?"
			else:
				verbal_query = "You want me to "+self.verbalize_apr_tuple(relatively_confident_about)+"?"
	
		return verbal_query,relatively_confident_about

	#process a restricted response from the user and update internal understanding based on expected contents of response only
	def process_system_initiative_response(self, user_utterances, utterance_class):
			
		print "process_system_initiative_response: current best understanding="+str(self.current_best_asp_understanding)+"; role requested="+str(self.role_requested) #DEBUG
		if (utterance_class == "yes"):
			self.request_type = None
			self.role_requested = None
			self.vocalize("I thought so")
			for role in self.asp_role_map:
				if (self.roles_relatively_confident_about[self.asp_role_map[role]] != None):
					self.current_asp_confidence[self.asp_role_map[role]][self.current_best_asp_understanding[self.asp_role_map[role]][0]] = 1
			return None
		elif (utterance_class == "no"):
			self.vocalize("Sorry I misunderstood",request_type="failed_guess",role_requested=None)
			if (self.current_best_asp_understanding[self.asp_role_map["action"]][0] != "query" and self.role_requested != None and self.current_best_asp_understanding[self.asp_role_map[self.role_requested]][0] != None): #the user is rejecting a clarification on an argument
				self.current_asp_confidence[self.asp_role_map[self.role_requested]][self.current_best_asp_understanding[self.asp_role_map[self.role_requested]][0]] = 0
			else: #the user is rejecting an assumption that the system has made; this is the same as a termination request
				#temporarily, we are rejecting when 'query' was rejecting too
				self.request_type = "terminate"
				self.role_requested = None
			return None
		elif (utterance_class == "nevermind"):
			self.request_type = "terminate"
			self.role_requested = None
			return None
			
		#if sure we're requesting confirmation and confirmation/denial was not given, need to do an exit to re-clarify
		if (self.roles_relatively_confident_about[self.asp_role_map[self.role_requested]] != None):
			self.vocalize("Let me get a handle on what you want, first.")
			return None
			
		#make a pass at understanding by parsing
		parses_in_utterance_understandings = []
		unmapped_words_in_utterance_understandings = []
		utterance_scores = []
		for user_utterance,utterance_score in user_utterances:
			utterance_scores.append(utterance_score)
			utterance_parses,unmapped_words_in_utterance_understanding = self.parse_utterance(user_utterance)
			parses_in_utterance_understandings.append(utterance_parses)
			unmapped_words_in_utterance_understandings.append(unmapped_words_in_utterance_understanding)
	
		#try to map each candidate parse into an ASP node
		asp_nodes = []
		highest_confidence_index = (0,None) #default to highest-confidence ASR for utterance/alignment
		highest_confidence_score = -sys.maxint
		for i in range(0,len(user_utterances)):
			parses = parses_in_utterance_understandings[i]
			sys.stderr.write("parses for understanding '"+user_utterances[i][0]+"'\n")
			sys.stderr.write(str(parses)+"\n")
			for j in range(0,len(parses)):
				asp_node = self.pta.answer_parse_with_asp(parses[j][0])
				if (asp_node != None and parses[j][1]+utterance_scores[i] > highest_confidence_score): #score is parse score plus ASR score to determine most confident parse
					highest_confidence_index = (i,j)
				if (type(asp_node) is list): #ambiguity; we will reduce our confidence in the extractions accordingly
					sys.stderr.write("DEBUG: asp translation yielded multiped valid instantiations for parse '"+parses[j][0]+"'\n")
					asp_nodes.append([i,j,asp_node])
				else:
					if (asp_node == None):
						sys.stderr.write("DEBUG: asp translation failed for parse '"+parses[j][0]+"'\n")
					else:
						sys.stderr.write("DEBUG: asp translation successful for parse '"+parses[j][0]+"'\n")
						asp_nodes.append([i,j,asp_node])
					
		#choose the "gold" user utterance for future parse (re)training and word alignment guessing
		print "DEBUG: highest confidence utterance: '"+str(user_utterances[highest_confidence_index[0]][0])+"'"
		if (highest_confidence_index[1] != None): #else, there will have been no successful translations so we will return None when that check is made
			highest_confidence_parse = parses_in_utterance_understandings[highest_confidence_index[0]][highest_confidence_index[1]][0]
		user_utterance = user_utterances[highest_confidence_index[0]][0]
		unmapped_words_in_utterance = unmapped_words_in_utterance_understandings[highest_confidence_index[0]]
	
		#update alignment guesses
		self.utterances_for_clarification.append(user_utterance)
		goal_utterances_plus_clarification_utterances = self.utterances_for_goal[:]
		goal_utterances_plus_clarification_utterances.extend(self.utterances_for_clarification)
		#self.update_unmapped_words_alignment_guesses(self.unmapped_words_from_utterances, goal_utterances_plus_clarification_utterances, unmapped_words_in_utterance)
	
		#if user used single-term response which we have a lexicon mapping for, we can take that as the answer
		if (user_utterance in self.known_words and self.known_words[user_utterance][1] in self.grounded_to_lexicon_map):
			self.current_asp_confidence[self.asp_role_map[self.role_requested]][self.known_words[user_utterance][1]] = self.min_confidence_to_accept
			return self.known_words[user_utterance][1] #return a known semantic parse for the utterance for re-training
			
		#after checking for single-term response and finding none, we know to note whatever was said as an alternative form of an upcoming, known clarification, so we record this
		if (self.role_requested in self.utterances_during_dialogue):
			self.utterances_during_dialogue[self.role_requested].append(user_utterance)
		else:
			self.utterances_during_dialogue[self.role_requested] = [user_utterance]
		
		#no parses
		if (len(asp_nodes) == 0):
			return None
		
		#update our confidence in various possible answers using an interpolation of parse confidence and asp ambiguity from multiple instantiation
		role_candidates = []
		for i in range(0,len(asp_nodes)):
			apr_tuple = self.get_apr_tuple_from_asp_node(asp_nodes[i][2])
			print "apr_tuple="+str(apr_tuple) #DEBUG
			if (apr_tuple[self.asp_role_map["action"]].keys()[0] == "query"): #if the tuple parsed into a query, the role requested will be put in the "patient" slot
				role_requested_location = self.asp_role_map["patient"]
			else:
				role_requested_location = self.asp_role_map[self.role_requested]
			for candidate in apr_tuple[role_requested_location]:
				if (candidate == None):
					continue
				confidence = apr_tuple[role_requested_location][candidate]
				role_candidates.append(candidate)
				if (candidate in self.current_asp_confidence[self.asp_role_map[self.role_requested]]):
					self.current_asp_confidence[self.asp_role_map[self.role_requested]][candidate] += (1-self.current_asp_confidence[self.asp_role_map[self.role_requested]][candidate])*confidence
				else:
					self.current_asp_confidence[self.asp_role_map[self.role_requested]][candidate] = confidence
					
		#decay those things not mentioned in clarification but involved in the concerned role
		for existing_candidates in self.current_asp_confidence[self.asp_role_map[self.role_requested]]:
			if (existing_candidates not in role_candidates):
				self.current_asp_confidence[self.asp_role_map[self.role_requested]][existing_candidates] = self.current_asp_confidence[self.asp_role_map[self.role_requested]][existing_candidates]*self.confidence_decay_rate

		return highest_confidence_parse

	#string together an asp goal state from an apr tuple
	def write_asp_goal_from_apr_tuple(self, apr):
		
		if (apr["action"] == "served"):
			asp_function = apr["action"]+"("+apr["recipient"].split(':')[0]+","+apr["patient"].split(':')[0]+",n)"
		elif (apr["action"] == "at"):
			asp_function = apr["action"]+"("+apr["recipient"].split(':')[0]+",n)"
		elif (apr["action"] == "query"):
			asp_function = apr["action"]+"("+apr["patient"]+")"
		else:
			sys.stderr.write("ERROR: unrecognized apr action '"+apr["action"]+"' in dialogue manager")
			
		return asp_function

	#start a command dialogue with the user
	def get_command_from_user(self, user_response=None):

		#greeting state
		self.current_best_asp_understanding = [[None,0],[None,0],[None,0]]
		self.current_asp_confidence = [{},{},{}]
		self.request_type = "user_initiative"
		self.role_requested = None
		self.utterances_for_goal = []
		self.utterances_for_clarification = []
		self.utterances_during_dialogue = {}
		self.dialogue_accepted_parses = {}
		#self.unmapped_words_from_utterances = []
		self.vocalize("How can I help?")
		
		#dialogue control loop continues until minimum confidence among arguments meets acceptable threshold
		confident_in_understanding = False
		while (confident_in_understanding == False):
			
			#user's turn, user initiative
			if (self.request_type == "user_initiative"):
				user_responses,user_response_class = self.get_user_response()
				best_parse_result = self.process_user_initiative_response(user_responses,user_response_class)
				if (best_parse_result != None):
					self.dialogue_accepted_parses["goal"] = best_parse_result
				if (self.request_type == "terminate"): #user gave up in response
					break
			
			#user's turn, system initiative
			elif (self.request_type == "system_initiative"):
				user_responses,user_response_class = self.get_user_response()
				best_parse_result = self.process_system_initiative_response(user_responses,user_response_class)
				print "best_parse_result = "+str(best_parse_result)+" for role "+str(self.role_requested) #DEBUG
				if (best_parse_result != None):
					self.dialogue_accepted_parses[self.role_requested] = best_parse_result
				if (self.request_type == "terminate"): #user rejected clarification, so we made a bad choice in what to believe
					self.request_type = "user_initiative"
					self.role_requested = None
					self.current_best_asp_understanding = [[None,0],[None,0],[None,0]] #start fresh
					self.current_asp_confidence = [{},{},{}] #start fresh
					#for role in self.asp_role_map: #decay all results
					#	for existing_candidates in self.current_asp_confidence[self.asp_role_map[role]]:
					#		self.current_asp_confidence[self.asp_role_map[role]][existing_candidates] = self.current_asp_confidence[self.asp_role_map[role]][existing_candidates]*self.confidence_decay_rate
						
			#update internal states based on user's response
			for role in self.asp_role_map: #get new best understanding
				max_role_conf,max_role_arg = self.max_argmax(self.current_asp_confidence[self.asp_role_map[role]])
				self.current_best_asp_understanding[self.asp_role_map[role]][0] = max_role_arg
				self.current_best_asp_understanding[self.asp_role_map[role]][1] = max_role_conf
			sys.stderr.write("DEBUG: current best asp understanding: "+str(self.current_best_asp_understanding)+"\n")
			sys.stderr.write("DEBUG: current current asp confidence: "+str(self.current_asp_confidence)+"\n")
			if (min([self.current_best_asp_understanding[self.asp_role_map[role]][1] for role in self.asp_role_map]) > self.min_confidence_to_accept):
				confident_in_understanding = True
				break
				
			#DEBUG
			for r in self.utterances_during_dialogue:
				print r+"\t"+str(self.utterances_during_dialogue[r])+"\t"+str(self.dialogue_accepted_parses[r] if r in self.dialogue_accepted_parses else None)
			#/DEBUG
			
			#system's turn
			vocalize_for_user,self.roles_relatively_confident_about = self.articulate_next_state()
			self.vocalize(vocalize_for_user)
			
		#note all unmapped words found so far for which we have any alignment guesses from this dialogue
		#self.update_all_unmapped_words_guesses(self.unmapped_words_from_utterances)
			
		#terminate state reached now, so determine what response to take
		if (confident_in_understanding == True):
		
			#reconstruct best parse if it is missing from components
			#NOTE: this may disrupt the parser, since the constructed parses will never involve lambda expressions; something to tune
			# if (best_parse_result == None):
# 				if (self.current_best_asp_understanding[self.asp_role_map["action"]][0] == "query"):
# 					pass #the semantic parser definitely won't benefit from loading a whole expression with its lambda answer
# 				else:
# 					action_name = "bring:a" if self.current_best_asp_understanding[self.asp_role_map["action"]][0] == "served" else "walk:a"
# 					best_parse_result = "(and:<t*,t> (action:<a,t> "+action_name+")"
# 					best_parse_result += " (actionrecipient:<a,<e,t>> "+action_name+" "+self.current_best_asp_understanding[self.asp_role_map["recipient"]][0]+")"
# 					if (self.current_best_asp_understanding[self.asp_role_map["patient"]][0] != False):
# 						best_parse_result += " (actionpatient:<a,<e,t>> "+action_name+" "+self.current_best_asp_understanding[self.asp_role_map["patient"]][0]+")"
# 					best_parse_result += ")"
			
			#note parse pairs induced from final, correct parse, if found, for each request type
			for r in self.dialogue_accepted_parses:
				if (r in self.utterances_during_dialogue):
					for utterance in self.utterances_during_dialogue[r]:
						self.utterance_parse_pairs.append([utterance,self.dialogue_accepted_parses[r]])
						sys.stderr.write("DEBUG: utterance_parse_pairs adding pair '"+utterance+"'->'"+self.dialogue_accepted_parses[r]+"'\n")
		
			#return the asp goal state generated
			if (self.current_best_asp_understanding[self.asp_role_map["action"]][0] == "query"):
				patient_ref = self.choose_referring_expression(self.current_best_asp_understanding[self.asp_role_map["patient"]][0],self.utterances_for_goal[-1])
				self.vocalize(patient_ref)
			else:
				self.request_type = None
				self.role_requested = None
				self.vocalize("Happy to help")
			f = open(self.log_filename,'a')
			f.write("\n")
			f.close()
			#apr_tuple_to_write_goal_from = {role:self.current_best_asp_understanding[self.asp_role_map[role]][0] for role in self.asp_role_map}
			apr_tuple_to_write_goal_from = {}
			for role in self.asp_role_map:
				apr_tuple_to_write_goal_from[role] = self.current_best_asp_understanding[self.asp_role_map[role]][0]
			return self.write_asp_goal_from_apr_tuple(apr_tuple_to_write_goal_from)
		
		else:
			self.vocalize("Sorry I couldn't understand.")
			f = open(self.log_filename,'a')
			f.write("\n")
			f.close()
			return None
			
	#generate a (series) of utterances in response to last user utterance contained in poll file, or start new dialogue		
	def get_command_from_user_offline(self):
	
		#check for existing state pickle to decide whether this is the first call for the session
		try:
			f = open(self.core_filename,'rb')
			starting = False
			f.close()
		except IOError:
			starting = True
	
		#first call
		if (starting == True):
			#greeting state
			self.current_best_asp_understanding = [[None,0],[None,0],[None,0]]
			self.current_asp_confidence = [{},{},{}]
			self.request_type = "user_initiative"
			self.role_requested = None
			self.utterances_for_goal = []
			self.utterances_for_clarification = []
			self.utterances_during_dialogue = {}
			self.dialogue_accepted_parses = {}
			self.vocalize("How can I help?")
			self.write_core_elements_to_pickle_and_shutdown()
			
		#subsequent call; load data and process response
		else:
			#load dialogue state
			self.load_core_elements_from_pickle()
			#get user response
			user_responses,user_response_class = self.get_user_response()
		
		#dialogue control loop continues until minimum confidence among arguments meets acceptable threshold
		confident_in_understanding = False
			
		#user's turn, user initiative
		if (self.request_type == "user_initiative"):
			best_parse_result = self.process_user_initiative_response(user_responses,user_response_class)
			if (best_parse_result != None):
				self.dialogue_accepted_parses["goal"] = best_parse_result
			if (self.request_type == "terminate"): #user gave up in response
				self.vocalize("Sorry I couldn't understand.")
				return False
		
		#user's turn, system initiative
		elif (self.request_type == "system_initiative"):
			best_parse_result = self.process_system_initiative_response(user_responses,user_response_class)
			print "best_parse_result = "+str(best_parse_result)+" for role "+str(self.role_requested) #DEBUG
			if (best_parse_result != None):
				self.dialogue_accepted_parses[self.role_requested] = best_parse_result
			if (self.request_type == "terminate"): #user rejected clarification, so we made a bad choice in what to believe
				self.request_type = "user_initiative"
				self.role_requested = None
				self.current_best_asp_understanding = [[None,0],[None,0],[None,0]] #start fresh
				self.current_asp_confidence = [{},{},{}] #start fresh
				#for role in self.asp_role_map: #decay all results
				#	for existing_candidates in self.current_asp_confidence[self.asp_role_map[role]]:
				#		self.current_asp_confidence[self.asp_role_map[role]][existing_candidates] = self.current_asp_confidence[self.asp_role_map[role]][existing_candidates]*self.confidence_decay_rate
					
		#update internal states based on user's response
		for role in self.asp_role_map: #get new best understanding
			max_role_conf,max_role_arg = self.max_argmax(self.current_asp_confidence[self.asp_role_map[role]])
			self.current_best_asp_understanding[self.asp_role_map[role]][0] = max_role_arg
			self.current_best_asp_understanding[self.asp_role_map[role]][1] = max_role_conf
		if (min([self.current_best_asp_understanding[self.asp_role_map[role]][1] for role in self.asp_role_map]) > self.min_confidence_to_accept):
			confident_in_understanding = True
			
		if (confident_in_understanding == False):
			#system's turn
			vocalize_for_user,self.roles_relatively_confident_about = self.articulate_next_state()
			self.vocalize(vocalize_for_user)
			self.write_core_elements_to_pickle_and_shutdown()

		else:
			
			#log parse pairs induced from final, correct parse, if found, for each request type
			f = open(self.alog_filename,'a')
			for r in self.dialogue_accepted_parses:
				if (r in self.utterances_during_dialogue):
					for utterance in self.utterances_during_dialogue[r]:
						f.write(utterance+"\n"+self.dialogue_accepted_parses[r]+"\n\n")
			f.close()
						
			#return the asp goal state generated
			if (self.current_best_asp_understanding[self.asp_role_map["action"]][0] == "query"):
				patient_ref = self.choose_referring_expression(self.current_best_asp_understanding[self.asp_role_map["patient"]][0],self.utterances_for_goal[-1])
				self.request_type = None
				self.role_requested = None
				self.vocalize(patient_ref)
				self.vocalize("Happy to help")
			else:
				self.request_type = None
				self.role_requested = None
				self.vocalize("Happy to help")
			f = open(self.log_filename,'a')
			f.write("\n")
			f.close()
			#apr_tuple_to_write_goal_from = {role:self.current_best_asp_understanding[self.asp_role_map[role]][0] for role in self.asp_role_map}
			apr_tuple_to_write_goal_from = {}
			for role in self.asp_role_map:
				apr_tuple_to_write_goal_from[role] = self.current_best_asp_understanding[self.asp_role_map[role]][0]
			return self.write_asp_goal_from_apr_tuple(apr_tuple_to_write_goal_from)
