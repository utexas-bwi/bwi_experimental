/**
 * \file  vi_example.cpp
 * \brief A simple example demonstrating how to use Value Iteration using the bwi_rl code base to solve for an agent
 *        trying to find a goal in 10x by 10x torus grid.
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2015, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 05/26/2015 05:00:28 PM piyushk $
 *
 **/

#include<boost/foreach.hpp>

#include <bwi_rl/planning/PredictiveModel.h>
#include <bwi_rl/planning/ValueIteration.h>
#include <bwi_rl/planning/VITabularEstimator.h>

#define GRID_SIZE 10

enum Action {
  UP,
  DOWN,
  LEFT,
  RIGHT
};

std::ostream& operator<<(std::ostream& stream, const Action& action) {
  if (action == UP) {
    stream << "Up";
  } else if (action == DOWN) {
    stream << "Down";
  } else if (action == LEFT) {
    stream << "Left";
  } else {
    stream << "Right";
  }
  return stream;
}

struct State {
  int x;
  int y;
  bool localized; 

  // boost serialize
  private:
    friend class boost::serialization::access;
    template <typename Archive> void serialize(Archive &ar, const unsigned int version) {
      ar & x;
      ar & y;
    }
};

bool operator<(const State& l, const State& r) {
  if (l.x < r.x) return true;
  if (l.x > r.x) return false;

  if (l.y < r.y) return true;
  if (l.y > r.y) return false;
}

bool operator==(const State& l, const State& r) {
  return (l.x == r.x && l.y == r.y);
}

std::ostream& operator<<(std::ostream& stream, const State& s) {
  stream << "(" << s.x << "," << s.y << ")";
  return stream;
}

class GridModel : public PredictiveModel<State, Action> {

  public:

    GridModel() {

      // Precache the complete state vector of 100 states.
      for (int x = 0; x < GRID_SIZE; ++x) {
        for (int y = 0; y < GRID_SIZE; ++y) {
          State s;
          s.x = x;
          s.y = y;
          complete_state_vector.push_back(s);
        }
      }

      // Precache the default action list (if state is not a terminal state)
      default_action_list.push_back(UP);
      default_action_list.push_back(DOWN);
      default_action_list.push_back(LEFT);
      default_action_list.push_back(RIGHT);
    }

    GridModel(std::string path_to_maps) {

        static_map = parseMap(path_to_maps + "static_map.txt"); 
        dynamic_map = parseMap(path_to_maps + "dynamic_map.txt"); 
        sunny_map = parseMap(path_to_maps + "sunny_map.txt"); 
        trap_map = parseMap(path_to_maps + "trap_map.txt"); 

        height = static_map.size();
        width = static_map[0].size(); 
        complete_state_vector.clear(); 
        for (int x = 0; x < height; x++) {
            for (int y = 0; y < width; y++) {
                State s;
                s.x = x; 
                s.y = y;
                s.localized = true; 
                complete_state_vector.push_back(s); 
            }
        }
        State s; 
        s.x = -1; 
        s.y = -1; 
        s.localized = false; 
        complete_state_vector.push_back(s);

        default_action_list.push_back(UP);
        default_action_list.push_back(DOWN);
        default_action_list.push_back(LEFT);
        default_action_list.push_back(RIGHT);
    }

    virtual bool isTerminalState(const State &state) const {
      return (state.x == GRID_SIZE/2 and state.y == GRID_SIZE/2);
    }

    virtual void getActionsAtState(const State &state, std::vector<Action>& actions) {
      actions.clear();
      if (!isTerminalState(state)) {
        actions = default_action_list;
      }
    };

    virtual void getStateVector(std::vector<State>& states) {
      states = complete_state_vector;
    }

    virtual void getTransitionDynamics(const State &state,
        const Action &action, std::vector<State> &next_states,
        std::vector<float> &rewards, std::vector<float> &probabilities) {
      next_states.clear();
      rewards.clear();
      probabilities.clear();
      if (!isTerminalState(state)) {

        // Transition Dynamics are only valid at a non terminal state
        State ns;
        ns.x = state.x;
        ns.y = (state.y == 0) ? GRID_SIZE - 1 : state.y - 1;
        next_states.push_back(ns);
        ns.x = state.x;
        ns.y = (state.y + 1) % GRID_SIZE;
        next_states.push_back(ns);
        ns.x = (state.x == 0) ? GRID_SIZE - 1 : state.x - 1;
        ns.y = state.y;
        next_states.push_back(ns);
        ns.x = (state.x + 1) % GRID_SIZE;
        ns.y = state.y;
        next_states.push_back(ns);

        rewards.push_back(-1);
        rewards.push_back(-1);
        rewards.push_back(-1);
        rewards.push_back(-1);

        float p_up = (action == UP) ? 0.7f : 0.1f;
        float p_down = (action == DOWN) ? 0.7f : 0.1f;
        float p_left = (action == LEFT) ? 0.7f : 0.1f;
        float p_right = (action == RIGHT) ? 0.7f : 0.1f;

        probabilities.push_back(p_up);
        probabilities.push_back(p_down);
        probabilities.push_back(p_left);
        probabilities.push_back(p_right);

      }

    };

    virtual std::string generateDescription(unsigned int indentation = 0) {
      return std::string("");
    }

    std::vector<std::vector<int>> parseMap(std::string map_file) {
        std::vector<std::vector<int>> ret;
        ifstream input_file(map_file);
        if (intput_file) {
            std::string str;
            vector<int> vec;
            while ( input_file >> str ) {
                if (str.find("\n" != str.end())) {
                    ret.push_back(vec);
                    vec.clear();
                } else {
                    vec.push_back(std::stoi(str));
                }
            }
        }
        return ret;
    }

  private:

    std::vector<State> complete_state_vector;
    std::vector<Action> default_action_list;
    int height, width; 
    std::vector<std::vector<int>> static_map, dynamic_map, sunny_map, trap_map; 
};

int main(int argc, char **argv) {
  boost::shared_ptr<PredictiveModel<State, Action> > model(new GridModel);
  boost::shared_ptr<VIEstimator<State, Action> > estimator(new VITabularEstimator<State, Action>);

  ValueIteration<State, Action> vi(model, estimator);
  std::cout << "Computing policy..." << std::endl;
  vi.computePolicy();

  std::vector<State> test_states;
  State test;
  test.x = 5;
  test.y = 2;
  test_states.push_back(test);
  test.x = 2;
  test.y = 5;
  test_states.push_back(test);
  test.x = 5;
  test.y = 8;
  test_states.push_back(test);
  test.x = 8;
  test.y = 5;
  test_states.push_back(test);

  BOOST_FOREACH(const State& s, test_states) {
    std::cout << "Best action at state " << s << " is " << vi.getBestAction(s) << std::endl;
    // std::vector<Action> actions;
    // model->getActionsAtState(s, actions);
    // BOOST_FOREACH(const Action& a, actions) {
    //   std::cout << "  Next state distributions after taking action " << a << std::endl;
    //   std::vector<State> ns;
    //   std::vector<float> r;
    //   std::vector<float> p;
    //   model->getTransitionDynamics(s, a, ns, r, p);
    //   int counter = 0;
    //   BOOST_FOREACH(const State& s2, ns) {
    //     std::cout << "    One possible next state is " << s2 << " with probability " << p[counter] << " and reward " << r[counter] << std::endl;
    //     ++counter;
    //   }
    // }
  }

  return 0;
}
