/*!
 * \author Teun Derksen
 * \date November 2013
 * \version 0.1
 */

#ifndef OMPL_BASE_BOUNDING_BOX_MOTION_VALIDATOR_
#define OMPL_BASE_BOUNDING_BOX_MOTION_VALIDATOR_

#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include "tue_map_3d/Map3D.h"



/** \brief Motion validator for a bounding box between 2 states */
class BoundingBoxMotionValidator : public ompl::base::MotionValidator{

public:
    BoundingBoxMotionValidator(ompl::base::SpaceInformationPtr si, octomap::OcTreeStamped* octomap) : MotionValidator(si)
    {
        octomap_ = octomap;
    }

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State*, double> &lastValid) const;

private:

    octomap::OcTreeStamped* octomap_;

    bool checkBoundingBox(const ompl::base::State *state) const;

    bool occupanyPoint(double x, double y, double z) const;

};
#endif

