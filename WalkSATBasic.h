//
// Created by cormac on 02/09/2020.
//

#ifndef INC_3DVIS_WALKSATBASIC_H
#define INC_3DVIS_WALKSATBASIC_H


#include <utility>
#include <vector>
#include <map>
#include <random>
#include <iostream>
#include "s_vis_zmq.hpp"

class WalkSATBasic {
private:
    std::vector<std::vector<long>> clauses;
    std::map<long, std::vector<long>> lit_clause;
    unsigned long n_vars;
    std::vector<long> random_interpretation;
    std::vector<long> true_sat_lit;
    s_vis_zmq::SVisZMQ *s_vis_zmq;

    void run_sat(unsigned max_flips_proportion = 4);

    long compute_broken(const std::vector<long> &clause, float omega = .4);

    void init_random_interpretation() {
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> dis(0.0, 1.0);
        for (unsigned long i = 1; i <= n_vars; i++) {
            // If random number is less than .5, return variable (true), otherwise, return -variable (false)
            random_interpretation[i] = dis(gen) < .5 ? i : -i;
        }
    }

    void init_true_sat_lit() {
        auto clausesBegin = clauses.begin();
        for (auto i = clausesBegin; i != clauses.end(); i++) {      // Iterate over clauses
            true_sat_lit[i - clausesBegin] = 0;
            for (auto j = i->begin(); j != i->end(); j++)               // For each literal in a clause
                if (random_interpretation[std::abs(*j)] == *j)          // If the literal is true in the interpretation
                    true_sat_lit[i - clausesBegin] += 1;                // Increment true_sat_lit array at clause index
        }
    }

    void update_tsl(long literal_to_flip) {
        for (auto clause_index : lit_clause[literal_to_flip])
            true_sat_lit[clause_index]++;
        for (auto clause_index : lit_clause[-literal_to_flip])
            true_sat_lit[clause_index]--;
    }

public:
    WalkSATBasic(std::vector<std::vector<long>> clauses, unsigned long nVars) :
            clauses(std::move(clauses)),
            n_vars(nVars),
            random_interpretation(n_vars + 1),
            true_sat_lit(this->clauses.size()),
            s_vis_zmq(nullptr) {
        auto clausesBegin = this->clauses.begin();
        for (auto i = clausesBegin; i != this->clauses.end(); i++)
            for (auto j = i->begin(); j != i->end(); j++)
                lit_clause[*j].push_back(i - clausesBegin);
    }

    int run();
};


#endif //INC_3DVIS_WALKSATBASIC_H
