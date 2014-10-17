import os,sys,operator,copy

class node:
	def __init__(self):
		self.function = None
		self.arguments = []

class parse_to_asp:

	def __init__(self, asp_dir, word_to_ontology_map):
	
		#store arguments
		self.asp_dir = asp_dir
		self.word_to_ontology_map = word_to_ontology_map
		
		#functions to known ASP fact names when the mapping is that straightforward
		self.functions_to_facts = {}
		self.functions_to_facts["selfat:<ro,t>"] = "at"
		#self.functions_to_facts["possesses:<pe,<e,t>>"] = "knowinside"
		self.functions_to_facts["possesses:<pe,<e,t>>"] = "inside"
		self.functions_to_facts["open:<do,t>"] = "open"
		#self.functions_to_facts["office:<of,t>"] = "office"
		self.functions_to_facts["office:<of,t>"] = "room"
		self.functions_to_facts["person:<pe,t>"] = "person"
		self.functions_to_facts["room:<ro,t>"] = "room"
		self.functions_to_facts["door:<do,t>"] = "door"
		self.functions_to_facts["hasdoor:<ro,<do,t>>"] = "hasdoor"
		self.functions_to_facts["equals:<e,<e,t>>"] = "equals"
		self.facts_mapped = []
		for key in self.functions_to_facts:
			if (self.functions_to_facts[key] not in self.facts_mapped):
				self.facts_mapped.append(self.functions_to_facts[key])
	
		#read in mapped facts from ASP
		f = open(os.path.join(asp_dir,'asp_known_data.txt'),'r')
		facts = f.readline().strip().split(' ')
		f.close()
		self.fact_functions = {} #indexed by (function name, arguments) tuple; valued at true/false
		self.fact_arguments = []
		for fact in facts:
			[function,arguments] = fact.split('(')
			if (function[0] == '-'):
				fact_true = False
				function = function[1:]
				continue #don't care about false facts; invalid keys default to this (may be refined later)
			else:
				fact_true = True
			if (function in self.facts_mapped):
				arguments = arguments[:-1].split(',') #remove trailing ')'
				try:
					timestep = int(arguments[-1])
					arguments = arguments[:-1] #remove timestep
				except:
					pass
				for arg in arguments:
					if (arg not in self.fact_arguments):
						self.fact_arguments.append(arg)
				if (len(arguments) == 1):
					key = (function,arguments[0])
				elif (len(arguments) == 2):
					key = (function,arguments[0],arguments[1])
				self.fact_functions[key] = fact_true
				
		#add explicit support for equals:<e,<e,t>> operator
		for arg in self.fact_arguments:
			self.fact_functions[("equals",arg,arg)] = True

	def answer_parse_with_asp(self, p):
	
		sys.stderr.write("raw parse:\n")
		sys.stderr.write(str(p)+"\n")
		if (p[0] != "("):
			p = "(the:<<e,t>,e> "+p+")"
		pt = self.parse_to_tree(p)
		sys.stderr.write("raw parse tree:\n")
		self.print_tree(pt)
		pt = self.compact_parse_tree(pt)
		sys.stderr.write("compacted parse tree:\n")
		self.print_tree(pt)
		if (type(pt) is not str):
			gpts = self.ground_lambdas_in_tree(pt)
			if (gpts == None): #failed to find satisfying lambdas when grounding
				return None
			if (len(gpts) > 0):
				sys.stderr.write("grounded parse tree(s):\n")
				for gpt in gpts:
					self.print_tree(gpt)
			else: #already grounded (no lambdas)
				gpts = [pt]
		else:
			gpts = [pt]
		asp_root = self.answer_parse_tree_with_asp(gpts)
		if (type(asp_root) is list and asp_root.count(None) == len(asp_root)): #a list of totally failed groundings
			return None
		if (type(asp_root) is list): #list response; ambiguity to resolve
			return asp_root
		if (asp_root != None and asp_root.function != None):
			return asp_root
		else:
			return None
		
	def print_tree(self, pt, d=0):
	
		for i in range(0,d):
			sys.stderr.write('\t')
		try:
			sys.stderr.write(pt.function+"\n")
			for arg in pt.arguments:
				self.print_tree(arg, d=d+1)
		except:
			sys.stderr.write(pt+"\n")

	def parse_to_tree(self, p):
		
		#create the root
		root = node()
		p = p[1:-1] #strip leading ( and trailing )
		root.function = p.split(' ')[0]
				
		#crawl through and recursively add sub trees
		num_open_parens = 0
		position_first_open_paren = None
		position_first_space = None
		for i in range(len(root.function),len(p)):
			if (position_first_open_paren == None and position_first_space == None and p[i] == ' '):
				position_first_space = i
			elif (position_first_open_paren == None and position_first_space != None and p[i] == ' '):
				root.arguments.append(p[position_first_space+1:i])
				position_first_space = i
			elif (p[i] == '('):
				if (position_first_open_paren == None):
					position_first_open_paren = i
					position_first_space = None
				num_open_parens += 1
			elif (p[i] == ')'):
				num_open_parens -= 1
				if (num_open_parens == 0):
					root.arguments.append(self.parse_to_tree(p[position_first_open_paren:i+1]))
					position_first_open_paren = None
					position_first_space = None
		if (position_first_open_paren == None and position_first_space != None):
			root.arguments.append(p[position_first_space+1:len(p)])
		
		#return the root with function and arguments now instantiated
		return root

	#collapses meaningless nodes like "the:<<e,t>,e>"
	def compact_parse_tree(self, pt, pt_parent=None):
		
		#base case, return leaves
		if (type(pt) is str):
			return pt
		
		#if a collapsable node, collapse and absorb child
		if (pt.function in ["the:<<e,t>,e>","exists:<<e,t>,t>"]):
			if (pt_parent != None):
				for i in range(0,len(pt_parent.arguments)):
					if (pt_parent.arguments[i] == pt):
						pt_parent.arguments[i] = pt.arguments[0]
			pt = pt.arguments[0]
		if (type(pt) is str):
			return pt
			
		#if useless argument, discard
		pt.arguments = [arg for arg in pt.arguments if arg != "true:t"]
				
		#recurse on children of (possibly collapsed) node
		for i in range(0,len(pt.arguments)):
			pt.arguments[i] = self.compact_parse_tree(pt.arguments[i],pt)
			
		return pt

	def ground_lambdas_in_tree(self, pt):
		#sys.stderr.write("ground_lambdas_in_tree:\trunning on tree with root '"+pt.function+"'\n") #DEBUG
		
		if (pt.function == "lambda"): #lambda takes two arguments: the variable name:type and a subtree of conditions
			#sys.stderr.write("ground_lambdas_in_tree:\tfound lambda root\n") #DEBUG
			
			lambdas = [pt.arguments[0].split(":")[0]]
			lambdas.extend(self.identify_lambdas_in_tree(pt.arguments[1]))
			
			inst_indices = [0 for i in range(0,len(lambdas))]
			satisfying_lambdas = []
			while True:
			
				#try current instantiation guess
				#sys.stderr.write("ground_lambdas_in_tree:\ttrying grounded values "+str([self.fact_arguments[inst_indices[j]] for j in range(0,len(lambdas))])+"\n") #DEBUG
				pt_inst = self.instantiate_lambdas_in_tree(copy.deepcopy(pt), lambdas, [self.fact_arguments[inst_indices[j]] for j in range(0,len(lambdas))])
				successful_grounding = self.evaluate_truth_of_instantiated_parse_tree(pt_inst)
				if (successful_grounding == True): #instantiate outermost lambda and note
					gpt = self.fact_arguments[inst_indices[0]]+":"+self.word_to_ontology_map[self.fact_arguments[inst_indices[0]]]
					satisfying_lambdas.append(gpt)
					
				#increment guess
				inst_indices[-1] += 1
				unsatisfiable = False
				if (len(lambdas) > 1):
					for i in range(1,len(lambdas)+1):
						if (inst_indices[-i] == len(self.fact_arguments)):
							if (i == len(lambdas)):
								unsatisfiable = True
								break #no combination satisfies
							inst_indices[-i] = 0
							inst_indices[-i-1] += 1
				else:
					if (inst_indices[0] == len(self.fact_arguments)):
						break #all combinations attempted
				if (unsatisfiable == True):
					break
						
			if (len(satisfying_lambdas) > 0):
				return satisfying_lambdas
			else:	
				sys.stderr.write("ground_lambdas_in_tree:\tfailed to find grounding for lambda root\n") #DEBUG
				return None
				
		else: #recurse to search for lambdas

			#sys.stderr.write("ground_lambdas_in_tree:\tgrounding subtrees under '"+pt.function+"'\n") #DEBUG
			gpts_satisfying = []
			for i in range(0,len(pt.arguments)):
				if (type(pt.arguments[i]) is not str):
					groundings = self.ground_lambdas_in_tree(pt.arguments[i])
					if (groundings == None): #could not get even one satisfying set of lambdas
						return None
					for grounding in groundings:
						gpt = copy.deepcopy(pt)
						gpt.arguments[i] = grounding
						gpts_satisfying.append(gpt)
			return gpts_satisfying

	def evaluate_truth_of_instantiated_parse_tree(self, pt):
	
		if (pt.function == "lambda"): #swallow
			#sys.stderr.write("evaluate_truth_of_instantiated_parse_tree:\tswallowing instantiated tree's lambda\n") #DEBUG
			pt = pt.arguments[1]
			return self.evaluate_truth_of_instantiated_parse_tree(pt)

		if (pt.function == "and:<t*,t>"): #need to validate all children
			#sys.stderr.write("evaluate_truth_of_instantiated_parse_tree:\tgrounding lambdas under 'and' condition\n") #DEBUG
			successful_grounding = True
			for i in range(0,len(pt.arguments)):
				successful_grounding = successful_grounding and self.evaluate_truth_of_instantiated_parse_tree(pt.arguments[i])
				if (successful_grounding == False):
					break #cease checking (short-circuit) if an unsatisfied sub-tree is found
			return successful_grounding
			
		if (pt.function == "not:<t,t>"): #need to validate and negate child
			return not self.evaluate_truth_of_instantiated_parse_tree(pt.arguments[0])

		#at condition leaves
		if (pt.function in self.functions_to_facts):
			#sys.stderr.write("evaluate_truth_of_instantiated_parse_tree:\tchecking grounding at condition leaf '"+pt.function+"'\n") #DEBUG
			try:
				if (len(pt.arguments) == 1):
					key = (self.functions_to_facts[pt.function], pt.arguments[0] if len(pt.arguments[0].split(":")) == 1 else pt.arguments[0].split(":")[0])
				elif (len(pt.arguments) == 2):
					key = (self.functions_to_facts[pt.function], pt.arguments[0] if len(pt.arguments[0].split(":")) == 1 else pt.arguments[0].split(":")[0], pt.arguments[1] if len(pt.arguments[1].split(":")) == 1 else pt.arguments[1].split(":")[0])
			except AttributeError: #malformed tree tried to query a function and failed
				#sys.stderr.write("evaluate_truth_of_instantiated_parse_tree:\tinvalid query on (malformed) parse tree:\n") #DEBUG
				#self.print_tree(pt) #DEBUG
				return False #invalid query / malformed parse tree
			try:
				#sys.stderr.write("evaluate_truth_of_instantiated_parse_tree:\tvalid key "+str(key)+" produced value "+str(self.fact_functions[key])+"\n") #DEBUG
				return self.fact_functions[key]
			except KeyError:
				#sys.stderr.write("evaluate_truth_of_instantiated_parse_tree:\tinvalid key "+str(key)+"\n") #DEBUG
				return False #invalid key
		else:
			sys.stderr.write("evaluate_truth_of_instantiated_parse_tree:\tunable to test truth of condition '"+pt.function+"'; defaulting to True\n") #DEBUG
			return True

	def identify_lambdas_in_tree(self, pt):
	
		if (pt.function == "lambda"):
			lambdas = [pt.arguments[0].split(":")[0]]
			lambdas.extend(self.identify_lambdas_in_tree(pt.arguments[1]))
			return lambdas
		else:
			lambdas = []
			for i in range(0,len(pt.arguments)):
				if (type(pt.arguments[i]) is not str):
					lambdas.extend(self.identify_lambdas_in_tree(pt.arguments[i]))
			return lambdas

	def instantiate_lambdas_in_tree(self, pt, lambdas_to_inst, insts):
	
		for i in range(0,len(pt.arguments)):
			if (type(pt.arguments[i]) is str):
				if (pt.arguments[i] in lambdas_to_inst):
					pt.arguments[i] = insts[lambdas_to_inst.index(pt.arguments[i])]
			else:
				pt.arguments[i] = self.instantiate_lambdas_in_tree(pt.arguments[i], lambdas_to_inst, insts)
		return pt

	#defines the types of queries the robot can handle; at present, single-entity answers and action commands
	def answer_parse_tree_with_asp(self, gpts): #takes a grounded parse tree
	
		#if root is a count, verbalize a return of the number of grounded trees
		if (type(gpts[0]) is not str and gpts[0].function == "count:<<e,t>,i>"):
			asp_root = node()
			asp_root.function = "query"
			asp_root.arguments = [str(len(gpts))]
			return asp_root
	
		#usual case; one satisfying tree with a non-count root
		elif (len(gpts) == 1):
			gpt = gpts[0]
	
			#if root is a string, just verbalize that
			if (type(gpt) is str):
				try:
					asp_root = node()
					asp_root.function = "query"
					asp_root.arguments = [gpt]
					return asp_root
				except:
					sys.stderr.write("answer_parse_tree_with_asp:\tunrecognized root ontological term '"+gpt+"'\n")
					return None
			
			#if "and" at root, check whether children form an action statement
			if (gpt.function == "and:<t*,t>"):
				for arg in gpt.arguments:
					if (arg.function == "action:<a,t>" and arg.arguments[0] == "bring:a"):
						return self.parse_bring_action(gpt.arguments)
					elif (arg.function == "action:<a,t>" and arg.arguments[0] == "walk:a"):
						return self.parse_walk_action(gpt.arguments)
										
			else:
				sys.stderr.write("answer_parse_tree_with_asp:\tunrecognized root function '"+gpt.function+"'\n")
				return None
				
		#multiple satisfying lambda assignments, not requesting count; need to resolve ambiguity
		else:
					
			root_and_verbalization_tuples = []
			for gpt in gpts:
				root_and_verbalization_tuples.append(self.answer_parse_tree_with_asp([gpt]))
			return root_and_verbalization_tuples

	def parse_bring_action(self, args):
	
		#identify ASP action to be taken from "bring" operation
		asp_node = node()
		asp_node.function = "served"
		asp_node.arguments = [None,None,"n"]
		for arg in args:
			if (arg.function == "actionpatient:<a,<e,t>>"):
				if (arg.arguments[0] != "bring:a"):
					sys.stderr.write("parse_bring_action:\taction in patient does not match action specified\n")
					return asp_node #sanity check; actions don't match within parse
				if (type(arg.arguments[1]) is not str):
					sys.stderr.write("parse_bring_action:\tnon-leaf argument '"+arg.arguments[1].function+"' found for patient\n")
					return asp_node
				if (arg.arguments[1].split(":")[0] == "me"): #the patient is the speaker, so we are to lead the speaker (ie, walk)
					asp_node.function = "at"
					asp_node.arguments = [None,"n"]
				elif (arg.arguments[1].split(":")[1] == "it"): #the patient is an item, so we are making a delivery
					asp_node.function = "served"
					asp_node.arguments[1] = arg.arguments[1]
				else:
					sys.stderr.write("parse_bring_action:\tpatient '"+arg.arguments[1]+"' not recognized\n")
					return asp_node
				break #after finding patient and deciding on action, clear to look for recipient
		if (asp_node.function == None):
			sys.stderr.write("parse_bring_action:\tno patient found\n")
				
		#determine the recipient of the action (either a person, for "served", or a room, for "at")
		for arg in args:
			if (arg.function == "actionrecipient:<a,<e,t>>"):
				if (arg.arguments[0] != "bring:a"):
					sys.stderr.write("parse_bring_action:\taction in recipient does not match action specified\n")
					return asp_node #sanity check; actions don't match within parse
				if (type(arg.arguments[1]) is not str):
					sys.stderr.write("parse_bring_action:\tnon-leaf argument '"+arg.arguments[1].function+"' found for recipient\n")
					return asp_node
				if (asp_node.function == "at" and (arg.arguments[1].split(":")[1] == "ro" or arg.arguments[1].split(":")[1] == "of")):
					asp_node.arguments[0] = arg.arguments[1]
				elif (asp_node.function == "served" and arg.arguments[1].split(":")[1] == "pe"):
					asp_node.arguments[0] = arg.arguments[1]
				else:
					sys.stderr.write("parse_bring_action:\trecipient '"+arg.arguments[1]+"' not recognized\n")
					return asp_node #recipient not understood
					
		return asp_node
		
	def parse_walk_action(self, args):
	
		asp_node = node()
		asp_node.function = "at"
		asp_node.arguments = [None,"n"]
		
		for arg in args:
			if (arg.function == "actionpatient:<a,<e,t>>"):
				sys.stderr.write("parse_walk_action:\taction patient found for 'walk'\n")
				return asp_node #sanity check; no patient should exist for walk action
			elif (arg.function == "actionrecipient:<a,<e,t>>"):
				if (arg.arguments[0] != "walk:a"):
					sys.stderr.write("parse_walk_action:\taction in recipient does not match action specified\n")
					return asp_node #sanity check; actions don't match within parse
				if (type(arg.arguments[1]) is not str):
					sys.stderr.write("parse_walk_action:\tnon-leaf argument '"+arg.arguments[1].function+"' found for recipient\n")
					return asp_node
				if (arg.arguments[1].split(":")[1] == "ro" or arg.arguments[1].split(":")[1] == "of"):
					asp_node.arguments[0] = arg.arguments[1]
				else:
					sys.stderr.write("parse_walk_action:\trecipient '"+arg.arguments[1]+"' not recognized\n")
					return asp_node #recipient not understood
					
		return asp_node
