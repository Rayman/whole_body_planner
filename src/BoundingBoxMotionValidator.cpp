#include "whole_body_planner/BoundingBoxMotionValidator.h"

#include <queue>

bool BoundingBoxMotionValidator::checkBoundingBox(const ompl::base::State *state) const{

    /// Convert the state to Real Vector space and check the bounding box
    const ompl::base::RealVectorStateSpace::StateType& coordinate = *state->as<ompl::base::RealVectorStateSpace::StateType>();

    if(!occupanyPoint((double)coordinate[0]+0.072, (double)coordinate[1], (double)coordinate[2]))
    {
        return false;
    }

    if(!occupanyPoint((double)coordinate[0], (double)coordinate[1]+0.072, (double)coordinate[2]))
    {
        return false;
    }

    if(!occupanyPoint((double)coordinate[0], (double)coordinate[1]-0.072, (double)coordinate[2]))
    {
        return false;
    }

    if(!occupanyPoint((double)coordinate[0], (double)coordinate[1], (double)coordinate[2]+0.072))
    {
        return false;
    }

    if(!occupanyPoint((double)coordinate[0], (double)coordinate[1], (double)coordinate[2]-0.072))
    {
        return false;
    }

    return true;
}

bool BoundingBoxMotionValidator::occupanyPoint(double x, double y, double z) const
{
    octomap::OcTreeNode* octree_node = octomap_->search((double)x,(double)y,(double)z);
    if (octree_node){
        if (octomap_->isNodeOccupied(octree_node)){
            return false;
        }
    }
    return true;
}

bool BoundingBoxMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    /// Assume motion starts in a valid state so s1 is valid
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }

    bool result = true;
    int nd = si_->getStateSpace()->validSegmentCount(s1,s2);

    /// Initialize the queue of test positions
    std::queue< std::pair<int, int> > pos;
    if (nd >= 2)
    {
        pos.push(std::make_pair(1, nd - 1));

        /// Temporary storage for the checked state
        ompl::base::State *test = si_->allocState();

        /// Repeatedly subdivide the path segment in the middle (and check the middle)
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            si_->getStateSpace()->interpolate(s1, s2, (double)mid / (double)nd, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }


            if(!checkBoundingBox(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.push(std::make_pair(x.first, mid - 1));
            if (x.second > mid)
                pos.push(std::make_pair(mid + 1, x.second));
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool BoundingBoxMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State*, double> &lastValid) const
{
    /// Assume motion starts in a valid state so s1 is valid
    bool result = true;
    int nd = si_->getStateSpace()->validSegmentCount(s1,s2);

    if (nd > 1)
    {
        /// Temporary storage for the checked state
        ompl::base::State *test = si_->allocState();

        for (int j = 1 ; j < nd ; ++j)
        {
            si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first)
                    si_->getStateSpace()->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first)
                si_->getStateSpace()->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
