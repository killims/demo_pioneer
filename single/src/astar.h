/*
 * astar.h
 *
 *  Created on: 14/01/2011
 *      Author: pablo
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <list>
#include <unordered_map>
#include <stdexcept>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "pqueue.h"
#include "state.h"

class astar {
	typedef double key_type;

	struct vertex {
		double g;
		state back_pointer;
		bool closed;

		inline vertex() :
			g(std::numeric_limits<double>::infinity()), 
					back_pointer(0, 0), closed(false) {
		}
	};

	typedef std::unordered_map<state, vertex, state_hash> space_type;
	typedef space_type::const_iterator const_iterator;
	typedef space_type::iterator iterator;
	//typedef pqueue<state, key_type, std::less<key_type>, state_hash>
	typedef pqueue<state, state_hash, key_type, std::less<key_type> >
			queue_type;

	std::list<state> m_path;

	bool have_work(const space_type& space, const state& s) const;

	void build_path(const space_type& space, const state& start,
			const state& goal);
public:
    static constexpr double epsilon = 1e-6;

	template<class Cost>
	void compute_one(space_type& space, queue_type& open, const state& goal,
			Cost& c) {
		state s = open.top().second;
		open.pop();

		iterator it = space.find(s);

		if (it == space.end())
			throw std::runtime_error("Bad iterator");

		it->second.closed = true;
		for (state::iterator p = s.begin_predecessors(); p
				!= s.end_predecessors(); ++p) {
			if (!obstacle(c, *p)) {
				double candidate_g = heuristic(*p, it->first) * cost(c,
						it->first) + it->second.g;
				iterator pred_it;

				pred_it = space.insert(std::make_pair(*p, vertex())).first;
				vertex& v = pred_it->second;
				if (v.g > candidate_g) {
					v.g = candidate_g;
					v.back_pointer = it->first;
					if (!v.closed)
						open.push(*p, pred_it->second.g + heuristic(*p, goal));
				}

			}
		}
	}

	template<class Cost>
	bool compute_path(const state& start, const state& goal, Cost& c) {
		space_type space;
		queue_type open;

    if (cost(c, start) == std::numeric_limits<double>::infinity() or cost(c, goal) == std::numeric_limits<double>::infinity())
    {
      m_path.clear();
      return true;
    }

		space[start].g = 0;
		open.push(start, heuristic(start, goal));

		while (have_work(space, goal))
			compute_one(space, open, goal, c);

		build_path(space, start, goal);

		return true;
	}

	template<class Cost>
	bool compute_path(const state& start, const state& goal, Cost& c,
			double timeout) {
		space_type space;
		queue_type open;

    if (cost(c, start) == std::numeric_limits<double>::infinity or cost(c, goal) == std::numeric_limits<double>::infinity)
    {
      m_path.clear();
      return true;
    }

		space[start].g = 0;
		open.push(start, heuristic(start, goal));

		boost::posix_time::ptime start_time =
				boost::posix_time::microsec_clock::local_time();
		while (have_work(space, goal)) {
			boost::posix_time::time_duration elapsed =
					boost::posix_time::microsec_clock::local_time()
							- start_time;
			if (elapsed.total_milliseconds() > timeout * 1e3) {
				std::cout << "timeout" << std::endl;
				m_path.clear();
				return false;
			}
			compute_one(space, open, goal, c);
		}

		build_path(space, start, goal);

		return true;
	}

	const std::list<state>& get_path() const;
};

inline const std::list<state>& astar::get_path() const {
	return m_path;
}

inline bool astar::have_work(const space_type& space, const state& goal) const {
	const_iterator goal_it = space.find(goal);
	return goal_it == space.end() or not goal_it->second.closed;
}

void astar::build_path(const space_type& space, const state& start,
		const state& goal) {
	state s = goal;
	while (s != start) {
		m_path.push_front(s);
		const_iterator it = space.find(s);
		if (it == space.end())
			throw std::runtime_error("Bad backpointer");
		s = it->second.back_pointer;
	}

	m_path.push_front(start);
}

#endif /* ASTAR_H_ */
