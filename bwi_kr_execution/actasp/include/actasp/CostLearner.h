#ifndef actasp_Cost_Learner_h__guard
#define actasp_Cost_Learner_h__guard

#include <actasp/AspFluent.h>

#include <fstream>
#include <map>
#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
namespace YAML {
  // The >> operator disappeared in yaml-cpp 0.5, so this function is
  // added to provide support for code written under the yaml-cpp 0.3 API.
  template<typename T>
  void operator >> (const YAML::Node& node, T& i)
  {
    i = node.as<T>();
  }
}
#endif

namespace actasp {

  class CostLearner {

    public:

      CostLearner(const std::map<std::string, int>& action_names_to_num_params_map) : 
        action_names_to_num_params_map(action_names_to_num_params_map) {}

      void initializeCostsFromValuesFile(const std::string& file) {

        std::ifstream fin(file.c_str());

        YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
        doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        parser.GetNextDocument(doc);
#endif

        for (size_t action_idx = 0; action_idx < doc.size(); ++action_idx) {
          std::string name;
          std::vector<std::string> vars;
          float cost;
          doc[action_idx]["name"] >> name;
          doc[action_idx]["cost"] >> cost;
          const YAML::Node &vars_node = doc[action_idx]["vars"];
          for (size_t var_idx = 0; var_idx < vars_node.size(); ++var_idx) {
            std::string var;
            vars_node[var_idx] >> var;
            vars.push_back(var);
          }
          AspFluent fluent(name, vars);
          costs[fluent] = cost;
        }

        fin.close();
      }

      void writeLuaFile(const std::string& file) const {

        std::ofstream fout(file.c_str());
        fout << "#begin_lua" << std::endl << std::endl;

        // Let's do some pre-process to optimize the lua file.
        std::map<std::string, std::vector<std::set<std::string> > > var_combos;
        for (std::map<AspFluent, float>::const_iterator it = costs.begin(); it != costs.end(); ++it) {

          std::string action_name = it->first.getName();

          // For all actions names, get unique set of values each parameter can take.
          const std::vector<std::string> params = it->first.getParameters();
          if (var_combos[action_name].size() != params.size()) {
            var_combos[action_name].resize(params.size());
          }
          for (int var_idx = 0; var_idx < params.size(); ++var_idx) {
            var_combos[action_name][var_idx].insert(params[var_idx]);
          }
        }

        // for (std::map<std::string, int>::const_iterator it = action_names_to_num_params_map.begin(); 
        //      it != action_names_to_num_params_map.end(); ++it) {
        //   std::string action_name = it->first;
        //   unsigned num_params = it->second;
        //   std::cout << "Action Name: " << action_name << std::endl;
        //   std::cout << "  Num Params: " << num_params << std::endl;
        //   if (var_combos.find(action_name) != var_combos.end()) {
        //     for (int i = 0; i < num_params; ++i) {
        //       std::set<std::string>& vc = var_combos[action_name][i];
        //       for (std::set<std::string>::const_iterator it = vc.begin(); it != vc.end(); ++it) {
        //         std::cout << "  Arg " << i << " has possible value " << *it << std::endl;
        //       }
        //     }
        //   }
        // }

        // Now we have a list of unique action names, as well as unique set of values that each parameter for a given
        // action can take. Create a separate cost function for each action.
        for (std::map<std::string, int>::const_iterator it = action_names_to_num_params_map.begin(); 
             it != action_names_to_num_params_map.end(); ++it) {

          // Function header.
          std::string action_name = it->first;
          unsigned num_params = it->second;

          fout << "function " << action_name << "_cost(";
          for (unsigned i = 0; i < num_params; ++i) {
            fout << "v" << i;
            if (i != num_params - 1) {
              fout << ", ";
            }
          }
          fout << ")" << std::endl;

          // Convert all vars to strings.
          for (unsigned i = 0; i < num_params; ++i) {
            fout << "\ts" << i << " = tostring(v" << i << ")" << std::endl;
          }

          // Print the various parameter combinations recursively.
          bool printDefault = true;
          if (var_combos.find(action_name) != var_combos.end()) {
            std::vector<std::string> empty_param_list(num_params);
            AspFluent empty_action(action_name, empty_param_list);
            if (num_params != 0) {
              printRecursiveVarList(fout, var_combos[action_name], empty_action);
            } else {
              // So we need to print the cost for a non-parametric action.
              std::map<AspFluent, float>::const_iterator cost_it = costs.find(empty_action);
              int value = (cost_it != costs.end()) ? round(cost_it->second) : 1;
              fout << "\treturn " << value;
              if (cost_it == costs.end()) {
                fout << " -- default value.";
              }
              fout << std::endl;
              printDefault = false;
            }
          } 

          if (printDefault) {
            fout << "\treturn 1 -- return 1 for any action not seen previously." << std::endl;
          }
          fout << "end" << std::endl << std::endl;

        }

        fout << "#end_lua." << std::endl;
        fout.close();
      }

      void writeValuesFile(const std::string& file) const {
        std::ofstream fout(file.c_str());
        for (std::map<AspFluent, float>::const_iterator it = costs.begin(); it != costs.end(); ++it) {
          fout << " - name: " << it->first.getName() << std::endl;
          fout << "   cost: " << it->second << std::endl;
          fout << "   vars: [";
          std::vector<std::string> params = it->first.getParameters();
          for (int param_idx = 0; param_idx < params.size(); ++param_idx) {
            fout << params[param_idx];
            if (param_idx != params.size() - 1) {
              fout << ", ";
            }
          }
          fout << "]" << std::endl;
        }
        fout.close();
      }

      virtual bool addSample(const AspFluent& action, 
                             const std::set<AspFluent>& currentState, 
                             bool actionFailed, 
                             float cost) = 0;

      virtual ~CostLearner() {}

    private:

      void printRecursiveVarList(std::ofstream& fout, 
                                 const std::vector<std::set<std::string> >& var_combo, 
                                 AspFluent& action, 
                                 int idx = 0, 
                                 std::string indentation = "\t") const {

        for (std::set<std::string>::const_iterator it = var_combo[idx].begin(); 
             it != var_combo[idx].end(); ++it) {
          fout << indentation << "if s" << idx << " == \"" << *it << "\" then" << std::endl;
          std::vector<std::string> action_params = action.getParameters();
          action_params[idx] = *it;
          AspFluent next_action(action.getName(), action_params);
          if (idx == var_combo.size() - 1) {
            std::map<AspFluent, float>::const_iterator cost_it = costs.find(next_action);
            int value = (cost_it != costs.end()) ? round(cost_it->second) : 1;
            fout << indentation + "\t" << "return " << value;
            if (cost_it == costs.end()) {
              fout << " -- combo not sampled.";
            }
            fout << std::endl;
          } else {
            printRecursiveVarList(fout, var_combo, next_action, idx + 1, indentation + "\t");
          }
          fout << indentation << "end" << std::endl;
        }
      }

    protected:

      std::map<std::string, int> action_names_to_num_params_map;
      std::map<AspFluent, float> costs;

  };
}

#endif /* end of include guard: actasp_Cost_Learner_h__guard */
