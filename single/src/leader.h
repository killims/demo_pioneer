/*
 * leader.h
 *
 *  Created on: 08-may-2009
 *      Author: pablo
 */

#ifndef LEADER_H_
#define LEADER_H_
////////////////////////////////////////////////////////////////////////////////
#include "ndproxy.h"
#include "robot.h"
#include "state.h"
//#include "dstar.h"

////////////////////////////////////////////////////////////////////////////////
class leader: public base_robot {
    public:
        //constructor
        leader();

        //constructor + setup
        leader(config& configuration);

        virtual void setup(config& configuration);

        //public members
        void set_goal(const gns::point& goal);
        void set_goal(int x, int y);

        bool goal_reached();


    protected:
        //goal
        gns::point m_goal;

        //goal reached
        bool m_goal_reached;

        virtual void new_laser();
        virtual void new_position();
        gns::point compute_subgoal(const gns::point& goal);
        void draw_path(const pixel& color);
        void draw_robot();

        std::ofstream m_goal_log;
        std::list<state> m_path;
};
////////////////////////////////////////////////////////////////////////////////
inline bool leader::goal_reached() {
    return m_goal_reached;
}
////////////////////////////////////////////////////////////////////////////////
#endif /* LEADER_H_ */
