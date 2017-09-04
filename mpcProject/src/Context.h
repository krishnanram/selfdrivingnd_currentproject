//
// Created by krramasw on 9/3/17.
//

#ifndef MPC_CONTEXT_H
#define MPC_CONTEXT_H


class Context {

    public:

        size_t N ;
        double dt ;
        double ref_v;
        double Lf ;
        double ref_cte ;
        double ref_epsi ;
        double nactuators_limit = 0;
        double delta_limit = 0 ;
        double acc_limit = 0;


};


#endif //MPC_CONTEXT_H
