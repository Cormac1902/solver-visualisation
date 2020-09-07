//
// Created by cormac on 02/09/2020.
//

#include <climits>
#include "WalkSATBasic.h"

void WalkSATBasic::run_sat(unsigned int max_flips_proportion) {
    auto max_flips = n_vars * max_flips_proportion;                     // Set maximum number of flips
    while (true) {                                                      // Loop forever
        init_random_interpretation();                                       // Get interpretation

        init_true_sat_lit();                                                // Get true_sat_lit array
        for (unsigned long i = 0; i < max_flips; i++) {                              // Iterate max_flips times

            std::vector<long> unsatisfied_clauses_index = {};

            auto tsl_begin = true_sat_lit.begin();
            for (auto j = tsl_begin;
                 j != true_sat_lit.end(); j++) {  // Iterate over true_sat_lit and check whether clause is unsatisfied
                auto pos = j - tsl_begin;
                if (!true_sat_lit[pos]) unsatisfied_clauses_index.push_back(pos);
            }

            if (unsatisfied_clauses_index.empty())                          // If all clauses are satisfied
                return;                               // Return successful interpretation

            std::random_device random_device;
            std::mt19937 engine{random_device()};
            std::uniform_int_distribution<int> dist(0, unsatisfied_clauses_index.size() - 1);
            auto clause_index = unsatisfied_clauses_index[dist(engine)];  // Choose a clause randomly

            auto unsatisfied_clause = clauses[clause_index];                // Get this clause from the clauses array

            auto lit_to_flip = compute_broken(unsatisfied_clause);          // Get random literal to change

            update_tsl(lit_to_flip);                                        // Update true_sat_lit array

            auto abs_lit = std::abs(lit_to_flip);

            random_interpretation[abs_lit] *= -1;             // Flip value of literal in interpretation

            s_vis_zmq->assign_variable(random_interpretation[abs_lit]);
            s_vis_zmq->variable_activity(abs_lit);
        }
    }
}

long WalkSATBasic::compute_broken(const std::vector<long> &clause, float omega) {
    auto break_min = LONG_MAX;                      // Largest number long can handle
    std::vector<long> best_literals = {};           // Declare best literals array
    for (auto literal : clause) {                   // Iterate over clause
        auto break_score = 0;                           // Set break score to 0

        for (auto clause_index : lit_clause[-literal])  // Checks which clauses the inverse of the literal is in
            if (true_sat_lit[clause_index] == 1)
                break_score++; // If a clause is currently satisfied by only one literal increment break score

        if (break_score < break_min) {              // If break score is less than current minimum break count
            break_min = break_score;                    // Set minimum break count to break score
            best_literals = {literal};                  // Set the best literals array to be the current one
        } else if (break_score == break_min) {      // If break score is equal to current minimum break count
            best_literals.push_back(literal);           // Append literal to best literals array
        }
    }

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0.0, 1.0);

    if (break_min != 0 && dis(gen) < omega)
        best_literals = clause; // If minimum break count isn't 0 and random is less than omega set the best literals to be the clause

    std::uniform_int_distribution<int> dist(0, best_literals.size() - 1);
    return best_literals[dist(gen)];  // Return a random literal from the best literals
}

int WalkSATBasic::run() {
    s_vis_zmq = new s_vis_zmq::SVisZMQ;

    run_sat();

    std::cout << "s SATISFIABLE" << std::endl;
    std::cout << "v ";
    for (auto i = random_interpretation.begin() + 1; i != random_interpretation.end(); i++) {
        std::cout << *i << " ";
    }
    std::cout << std::endl;

    s_vis_zmq->send_start_interactor();

    return 1;
}