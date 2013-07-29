#include "whole_body_planner/VisibilityBasedValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"

ompl::base::VisibilityBasedValidStateSampler::VisibilityBasedValidStateSampler(const SpaceInformation *si) :
    ValidStateSampler(si), sampler_(si->allocStateSampler())
{
    name_ = "visibilitybased";
}

bool ompl::base::VisibilityBasedValidStateSampler::sample(State *state)
{
    unsigned int attempts = 0;

    // Used to determine if a sample is valid
    bool valid = false;

    // Used to determine if a sample is added to the list of connectors and guards
    bool valid_state = false;

    do
    {
        sampler_->sampleUniform(state);
        valid = si_->isValid(state);
        ++attempts;
    } while (!valid && attempts < attempts_);

    /*
        First 2 states are guards by default
        ToDo MAKE THIS ADAPTIVE
    */

    if (guards_.size()<2){
        storeGuard(state);
    }
    else {
        no_visible_states = checkVis(state);

        /// {GUARD : CONNECTOR : DISGARDED}
        if(no_visible_states==0){       //Guard
            storeGuard(state);
            valid_state = true;
        }
        else if(no_visible_states==2){  //Connector (ToDo investigate if 2 is the magic number)
            storeConnector(state);
            valid_state = true;
        }
        else{                           //Disgarded
            valid_state = false;
        }
    }
    return valid_state;
}

bool ompl::base::VisibilityBasedValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    unsigned int attempts = 0;
    bool valid = false;
    bool valid_state = false;
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        valid = si_->isValid(state);
        ++attempts;
    } while (!valid && attempts < attempts_);

    if (guards_.size()<2){
        storeGuard(state);
    }
    else {
        no_visible_states = checkVis(state);
        //std::cout<<"I can see "<<no_visible_states<<" visibile states."<<std::endl;


        /// {GUARD : CONNECTOR : DISGARDED}
        if(no_visible_states==0){       //Guard
            storeGuard(state);
            valid_state = true;
        }
        else if(no_visible_states==2){  //Connector
            storeConnector(state);
            valid_state = true;
        }
        else{                           //Disgarded
            valid_state = false;
        }
    }
    return valid_state;
}

unsigned int ompl::base::VisibilityBasedValidStateSampler::checkVis(State *state)
{
    // Count the number of visibile states
    unsigned int count = 0;
    for (std::size_t i = 0 ; i < container_.size() ; ++i)
    {
        // Preliminary check to see wether stored states are still valid
        if(si_->isValid(state) && si_->isValid(container_[i])){

            // Check visibility between two states
            if(si_->checkMotion(state,container_[i])){
                ++count;
            }
        }
        else{
        }

    }
    return count;

}

void ompl::base::VisibilityBasedValidStateSampler::storeGuard(State *state)
{
    State *temp = si_->allocState();
    si_->copyState(temp,state);
    guards_.push_back(temp);
    container_.push_back(temp);

    //std::cout<<guards_.size()<<" guards!"<<std::endl;
}

void ompl::base::VisibilityBasedValidStateSampler::storeConnector(State *state)
{
    State *temp = si_->allocState();
    si_->copyState(temp,state);
    connectors_.push_back(temp);
    container_.push_back(temp);

    //std::cout<<connectors_.size()<<" connectors!"<<std::endl;
}
