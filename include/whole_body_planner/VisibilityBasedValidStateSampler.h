/*!
 * \author Teun Derksen
 * \date April 2013
 * \version 0.1
 */

#ifndef OMPL_BASE_SAMPLERS_VISIBILITY_BASED_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_VISIBILITY_BASED_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"


namespace ompl
{
    namespace base
    {

        /** \brief Generate valid samples using uniform sampling and check their visibility based on:
          T. Simeon, J.-P. Laumond and C. Nissoux, Visibility-based probablistic roadmaps for motion planning, 2000*/
        class VisibilityBasedValidStateSampler : public ValidStateSampler
        {
        public:

            /** \brief Constructor */
            VisibilityBasedValidStateSampler(const SpaceInformation *si);

            /** \brief Deconstructor */
            ~VisibilityBasedValidStateSampler(void)
            {
            }

            /** \brief Get a sample from the statespace */
            bool sample(State *state);

            /** \brief Get a sample from the statespace near a specified state */
            bool sampleNear(State *state, const State *near, const double distance);

            /** \brief Check the visibility of the sampled state to the guards and connectors */
            unsigned int checkVis(State *state);

            /** \brief Store the state to the guard list */
            void storeGuard(State *state);

            /** \brief Store the state to the connector list*/
            void storeConnector(State *state);


        protected:

            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            /** \brief The list of maintained guard states */
            std::vector<const State*> guards_;

            /** \brief The list of maintained connector states */
            std::vector<const State*> connectors_;

            /** \brief The list of maintained guards and connectors states */
            std::vector<const State*> container_;

            /** \brief The number of visible states to the sampled state */
            unsigned int no_visible_states;

        };

    }
}


#endif
