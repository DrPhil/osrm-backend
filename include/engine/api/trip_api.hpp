#ifndef ENGINE_API_TRIP_HPP
#define ENGINE_API_TRIP_HPP

#include "engine/api/route_api.hpp"
#include "engine/api/trip_parameters.hpp"

#include "engine/datafacade/datafacade_base.hpp"

#include "engine/internal_route_result.hpp"

#include "guidance/turn_instruction.hpp"

#include "util/integer_range.hpp"

namespace osrm::engine::api
{

// Helper function to determine if a maneuver involves a significant direction change
// based on the bearing change. This is more robust than relying on maneuver types
// which can vary based on the profile.
inline bool hasSignificantBearingChange(short bearing_before, short bearing_after)
{
    // Calculate the angular difference (handling wrap-around at 360)
    int diff = std::abs(static_cast<int>(bearing_after) - static_cast<int>(bearing_before));
    if (diff > 180)
    {
        diff = 360 - diff;
    }
    // Consider anything more than 20 degrees as a significant direction change
    // This threshold catches corners while ignoring slight curves
    return diff > 20;
}

// Helper function to determine if a maneuver is a decision point that affects routing
// These are maneuvers where the driver must make a choice that affects the path
inline bool isRouteDecisionPoint(const osrm::guidance::TurnInstruction &instruction,
                                  short bearing_before,
                                  short bearing_after)
{
    using namespace osrm::guidance::TurnType;

    // First check by maneuver type - these are definite decision points
    switch (instruction.type)
    {
    // These are actual decision points that affect the route
    case Turn:
    case Merge:
    case OnRamp:
    case OffRamp:
    case Fork:
    case EndOfRoad:
    case EnterRoundabout:
    case EnterAndExitRoundabout:
    case EnterRotary:
    case EnterAndExitRotary:
    case EnterRoundaboutIntersection:
    case EnterAndExitRoundaboutIntersection:
    case EnterRoundaboutAtExit:
    case ExitRoundabout:
    case EnterRotaryAtExit:
    case ExitRotary:
    case EnterRoundaboutIntersectionAtExit:
    case ExitRoundaboutIntersection:
    case Sliproad:
        return true;

    // For Continue and NewName, check if there's a significant bearing change
    // This catches corners that might not be classified as turns
    case Continue:
    case NewName:
        return hasSignificantBearingChange(bearing_before, bearing_after);

    // These don't affect the route choice
    case Notification: // mode changes, restrictions
    case NoTurn:       // mid-segment
    case Suppressed:   // suppressed turn
    case StayOnRoundabout:
    case Invalid:
    default:
        return false;
    }
}

class TripAPI final : public RouteAPI
{
  public:
    TripAPI(const datafacade::BaseDataFacade &facade_, const TripParameters &parameters_)
        : RouteAPI(facade_, parameters_), parameters(parameters_)
    {
    }
    void MakeResponse(const std::vector<std::vector<NodeID>> &sub_trips,
                      const std::vector<InternalRouteResult> &sub_routes,
                      const std::vector<PhantomNodeCandidates> &candidates,
                      osrm::engine::api::ResultT &response) const
    {
        BOOST_ASSERT(sub_trips.size() == sub_routes.size());

        if (std::holds_alternative<flatbuffers::FlatBufferBuilder>(response))
        {
            auto &fb_result = std::get<flatbuffers::FlatBufferBuilder>(response);
            MakeResponse(sub_trips, sub_routes, candidates, fb_result);
        }
        else
        {
            auto &json_result = std::get<util::json::Object>(response);
            MakeResponse(sub_trips, sub_routes, candidates, json_result);
        }
    }
    void MakeResponse(const std::vector<std::vector<NodeID>> &sub_trips,
                      const std::vector<InternalRouteResult> &sub_routes,
                      const std::vector<PhantomNodeCandidates> &candidates,
                      flatbuffers::FlatBufferBuilder &fb_result) const
    {
        auto data_timestamp = facade.GetTimestamp();
        flatbuffers::Offset<flatbuffers::String> data_version_string;
        if (!data_timestamp.empty())
        {
            data_version_string = fb_result.CreateString(data_timestamp);
        }

        auto response = MakeFBResponse(sub_routes,
                                       fb_result,
                                       [this, &fb_result, &sub_trips, &candidates]()
                                       { return MakeWaypoints(fb_result, sub_trips, candidates); });

        if (!data_timestamp.empty())
        {
            response->add_data_version(data_version_string);
        }
        fb_result.Finish(response->Finish());
    }
    void MakeResponse(const std::vector<std::vector<NodeID>> &sub_trips,
                      const std::vector<InternalRouteResult> &sub_routes,
                      const std::vector<PhantomNodeCandidates> &candidates,
                      util::json::Object &response) const
    {
        auto number_of_routes = sub_trips.size();
        util::json::Array routes;
        routes.values.reserve(number_of_routes);

        // If route_points is requested, we need to collect route points from all trips
        util::json::Array all_route_points;

        for (auto index : util::irange<std::size_t>(0UL, sub_trips.size()))
        {
            if (parameters.route_points)
            {
                // When route_points is enabled, we need to compute steps to extract maneuver
                // locations
                auto route_with_points =
                    MakeRouteWithPoints(sub_routes[index].leg_endpoints,
                                        sub_routes[index].unpacked_path_segments,
                                        sub_routes[index].source_traversed_in_reverse,
                                        sub_routes[index].target_traversed_in_reverse,
                                        all_route_points);
                routes.values.push_back(std::move(route_with_points));
            }
            else
            {
                auto route = MakeRoute(sub_routes[index].leg_endpoints,
                                       sub_routes[index].unpacked_path_segments,
                                       sub_routes[index].source_traversed_in_reverse,
                                       sub_routes[index].target_traversed_in_reverse);
                routes.values.push_back(std::move(route));
            }
        }
        if (!parameters.skip_waypoints)
        {
            response.values.emplace("waypoints", MakeWaypoints(sub_trips, candidates));
        }
        response.values.emplace("trips", std::move(routes));
        if (parameters.route_points)
        {
            response.values.emplace("route_points", std::move(all_route_points));
        }
        response.values.emplace("code", "Ok");
        auto data_timestamp = facade.GetTimestamp();
        if (!data_timestamp.empty())
        {
            response.values.emplace("data_version", data_timestamp);
        }
    }

  protected:
    // FIXME this logic is a little backwards. We should change the output format of the
    // trip plugin routing algorithm to be easier to consume here.

    // Creates a route and extracts route points (maneuver locations) that can recreate the route
    util::json::Object
    MakeRouteWithPoints(const std::vector<PhantomEndpoints> &leg_endpoints,
                        const std::vector<std::vector<PathData>> &unpacked_path_segments,
                        const std::vector<bool> &source_traversed_in_reverse,
                        const std::vector<bool> &target_traversed_in_reverse,
                        util::json::Array &route_points_out) const
    {
        // We need to compute legs with steps to extract maneuver locations
        auto legs_info = MakeLegsWithSteps(leg_endpoints,
                                           unpacked_path_segments,
                                           source_traversed_in_reverse,
                                           target_traversed_in_reverse);
        std::vector<guidance::RouteLeg> &legs = legs_info.first;
        std::vector<guidance::LegGeometry> &leg_geometries = legs_info.second;

        auto route = guidance::assembleRoute(legs);
        std::optional<util::json::Value> json_overview = MakeGeometry(MakeOverview(leg_geometries));

        // Extract route points from maneuver locations
        // These are the minimal set of waypoints that would recreate this route
        // We only include decision points (turns, forks, etc.) not informational maneuvers
        util::json::Array trip_route_points;

        // Track last added location to avoid duplicates
        std::optional<util::Coordinate> last_added_location;

        auto add_route_point = [&](const util::Coordinate &coord)
        {
            // Skip if this is the same location as the last added point
            if (last_added_location &&
                last_added_location->lon == coord.lon &&
                last_added_location->lat == coord.lat)
            {
                return;
            }

            util::json::Object point;
            util::json::Array location;
            location.values.push_back(
                util::json::Number{util::toFloating(coord.lon).__value});
            location.values.push_back(
                util::json::Number{util::toFloating(coord.lat).__value});
            point.values.emplace("location", std::move(location));
            trip_route_points.values.push_back(std::move(point));
            last_added_location = coord;
        };

        for (std::size_t leg_idx = 0; leg_idx < legs.size(); ++leg_idx)
        {
            const auto &leg = legs[leg_idx];
            for (std::size_t step_idx = 0; step_idx < leg.steps.size(); ++step_idx)
            {
                const auto &step = leg.steps[step_idx];

                // Always include depart steps - these are the trip waypoints
                // that must be visited to recreate the route
                if (step.maneuver.waypoint_type == guidance::WaypointType::Depart)
                {
                    add_route_point(step.maneuver.location);
                    continue;
                }

                // Skip arrive steps - we'll add the final destination separately
                if (step.maneuver.waypoint_type == guidance::WaypointType::Arrive)
                {
                    continue;
                }

                // For intermediate steps within a leg, only include those that
                // are actual decision points (turns, forks, etc.) or have
                // significant bearing changes
                if (isRouteDecisionPoint(step.maneuver.instruction,
                                         step.maneuver.bearing_before,
                                         step.maneuver.bearing_after))
                {
                    add_route_point(step.maneuver.location);
                }
            }
        }

        // Add the final destination point from the last leg's last step
        if (!legs.empty() && !legs.back().steps.empty())
        {
            const auto &final_step = legs.back().steps.back();
            add_route_point(final_step.maneuver.location);
        }

        // Add this trip's route points to the overall collection
        route_points_out.values.push_back(std::move(trip_route_points));

        // Build step geometries if the user requested steps
        std::vector<util::json::Value> step_geometries;
        if (parameters.steps)
        {
            const auto total_step_count =
                std::accumulate(legs.begin(),
                                legs.end(),
                                0,
                                [](const auto &v, const auto &leg) { return v + leg.steps.size(); });
            step_geometries.reserve(total_step_count);

            for (const auto idx : util::irange<std::size_t>(0UL, legs.size()))
            {
                auto &leg_geometry = leg_geometries[idx];

                std::transform(
                    legs[idx].steps.begin(),
                    legs[idx].steps.end(),
                    std::back_inserter(step_geometries),
                    [this, &leg_geometry](const guidance::RouteStep &step)
                    {
                        if (parameters.geometries == RouteParameters::GeometriesType::Polyline)
                        {
                            return static_cast<util::json::Value>(json::makePolyline<100000>(
                                leg_geometry.locations.begin() + step.geometry_begin,
                                leg_geometry.locations.begin() + step.geometry_end));
                        }

                        if (parameters.geometries == RouteParameters::GeometriesType::Polyline6)
                        {
                            return static_cast<util::json::Value>(json::makePolyline<1000000>(
                                leg_geometry.locations.begin() + step.geometry_begin,
                                leg_geometry.locations.begin() + step.geometry_end));
                        }

                        BOOST_ASSERT(parameters.geometries ==
                                     RouteParameters::GeometriesType::GeoJSON);
                        return static_cast<util::json::Value>(json::makeGeoJSONGeometry(
                            leg_geometry.locations.begin() + step.geometry_begin,
                            leg_geometry.locations.begin() + step.geometry_end));
                    });
            }
        }

        // Handle annotations
        std::vector<util::json::Object> annotations;
        auto requested_annotations = parameters.annotations_type;
        if (parameters.annotations &&
            (parameters.annotations_type == RouteParameters::AnnotationsType::None))
        {
            requested_annotations = RouteParameters::AnnotationsType::All;
        }

        if (requested_annotations != RouteParameters::AnnotationsType::None)
        {
            for (const auto idx : util::irange<std::size_t>(0UL, leg_geometries.size()))
            {
                auto &leg_geometry = leg_geometries[idx];
                util::json::Object annotation;

                if (requested_annotations & RouteParameters::AnnotationsType::Speed)
                {
                    double prev_speed = 0;
                    annotation.values.emplace(
                        "speed",
                        GetAnnotations(leg_geometry,
                                       [&prev_speed](const guidance::LegGeometry::Annotation &anno)
                                       {
                                           if (anno.duration < std::numeric_limits<double>::min())
                                           {
                                               return prev_speed;
                                           }
                                           else
                                           {
                                               auto speed =
                                                   std::round(anno.distance / anno.duration * 10.) /
                                                   10.;
                                               prev_speed = speed;
                                               return util::json::clamp_float(speed);
                                           }
                                       }));
                }

                if (requested_annotations & RouteParameters::AnnotationsType::Duration)
                {
                    annotation.values.emplace(
                        "duration",
                        GetAnnotations(leg_geometry,
                                       [](const guidance::LegGeometry::Annotation &anno)
                                       { return anno.duration; }));
                }
                if (requested_annotations & RouteParameters::AnnotationsType::Distance)
                {
                    annotation.values.emplace(
                        "distance",
                        GetAnnotations(leg_geometry,
                                       [](const guidance::LegGeometry::Annotation &anno)
                                       { return anno.distance; }));
                }
                if (requested_annotations & RouteParameters::AnnotationsType::Weight)
                {
                    annotation.values.emplace(
                        "weight",
                        GetAnnotations(leg_geometry,
                                       [](const guidance::LegGeometry::Annotation &anno)
                                       { return anno.weight; }));
                }
                if (requested_annotations & RouteParameters::AnnotationsType::Datasources)
                {
                    annotation.values.emplace(
                        "datasources",
                        GetAnnotations(leg_geometry,
                                       [](const guidance::LegGeometry::Annotation &anno)
                                       { return anno.datasource; }));
                }
                if (requested_annotations & RouteParameters::AnnotationsType::Nodes)
                {
                    util::json::Array nodes;
                    nodes.values.reserve(leg_geometry.node_ids.size());
                    for (const auto node_id : leg_geometry.node_ids)
                    {
                        nodes.values.push_back(
                            static_cast<std::uint64_t>(facade.GetOSMNodeIDOfNode(node_id)));
                    }
                    annotation.values.emplace("nodes", std::move(nodes));
                }
                if (requested_annotations & RouteParameters::AnnotationsType::Datasources)
                {
                    const auto MAX_DATASOURCE_ID = 255u;
                    util::json::Object metadata;
                    util::json::Array datasource_names;
                    for (auto i = 0u; i < MAX_DATASOURCE_ID; i++)
                    {
                        const auto name = facade.GetDatasourceName(i);
                        if (name.empty())
                            break;
                        datasource_names.values.push_back(std::string(facade.GetDatasourceName(i)));
                    }
                    metadata.values.emplace("datasource_names", datasource_names);
                    annotation.values.emplace("metadata", metadata);
                }

                annotations.push_back(std::move(annotation));
            }
        }

        // If user didn't request steps, clear them before building the route
        if (!parameters.steps)
        {
            for (auto &leg : legs)
            {
                leg.steps.clear();
            }
        }

        auto result = json::makeRoute(route,
                                      json::makeRouteLegs(std::move(legs),
                                                          std::move(step_geometries),
                                                          std::move(annotations)),
                                      std::move(json_overview),
                                      facade.GetWeightName());

        return result;
    }

    // Similar to RouteAPI::MakeLegs but always computes steps for route point extraction
    std::pair<std::vector<guidance::RouteLeg>, std::vector<guidance::LegGeometry>>
    MakeLegsWithSteps(const std::vector<PhantomEndpoints> &leg_endpoints,
                      const std::vector<std::vector<PathData>> &unpacked_path_segments,
                      const std::vector<bool> &source_traversed_in_reverse,
                      const std::vector<bool> &target_traversed_in_reverse) const
    {
        auto result =
            std::make_pair(std::vector<guidance::RouteLeg>(), std::vector<guidance::LegGeometry>());
        auto &legs = result.first;
        auto &leg_geometries = result.second;
        auto number_of_legs = leg_endpoints.size();
        legs.reserve(number_of_legs);
        leg_geometries.reserve(number_of_legs);

        for (auto idx : util::irange<std::size_t>(0UL, number_of_legs))
        {
            const auto &phantoms = leg_endpoints[idx];
            const auto &path_data = unpacked_path_segments[idx];

            const bool reversed_source = source_traversed_in_reverse[idx];
            const bool reversed_target = target_traversed_in_reverse[idx];

            auto leg = guidance::assembleLeg(facade,
                                             path_data,
                                             phantoms.source_phantom,
                                             phantoms.target_phantom,
                                             reversed_target);

            guidance::LegGeometry leg_geometry;

            // Always compute geometry and steps for route points extraction
            leg_geometry = guidance::assembleGeometry(BaseAPI::facade,
                                                      path_data,
                                                      phantoms.source_phantom,
                                                      phantoms.target_phantom,
                                                      reversed_source,
                                                      reversed_target);

            leg.summary = guidance::assembleSummary(
                facade, path_data, phantoms.target_phantom, reversed_target);

            auto steps = guidance::assembleSteps(BaseAPI::facade,
                                                 path_data,
                                                 leg_geometry,
                                                 phantoms.source_phantom,
                                                 phantoms.target_phantom,
                                                 reversed_source,
                                                 reversed_target);

            guidance::applyOverrides(BaseAPI::facade, steps, leg_geometry);
            steps = guidance::collapseSegregatedTurnInstructions(std::move(steps));
            guidance::trimShortSegments(steps, leg_geometry);
            leg.steps = guidance::handleRoundabouts(std::move(steps));
            leg.steps = guidance::collapseTurnInstructions(std::move(leg.steps));
            leg.steps = guidance::anticipateLaneChange(std::move(leg.steps));
            leg.steps = guidance::buildIntersections(std::move(leg.steps));
            leg.steps = guidance::suppressShortNameSegments(std::move(leg.steps));
            leg.steps = guidance::assignRelativeLocations(std::move(leg.steps),
                                                          leg_geometry,
                                                          phantoms.source_phantom,
                                                          phantoms.target_phantom);
            leg_geometry = guidance::resyncGeometry(std::move(leg_geometry), leg.steps);

            leg_geometries.push_back(std::move(leg_geometry));
            legs.push_back(std::move(leg));
        }
        return result;
    }

    struct TripIndex
    {
        TripIndex() = default;

        TripIndex(unsigned sub_trip_index_, unsigned point_index_)
            : sub_trip_index(sub_trip_index_), point_index(point_index_)
        {
        }

        unsigned sub_trip_index = std::numeric_limits<unsigned>::max();
        unsigned point_index = std::numeric_limits<unsigned>::max();

        bool NotUsed()
        {
            return sub_trip_index == std::numeric_limits<unsigned>::max() &&
                   point_index == std::numeric_limits<unsigned>::max();
        }
    };

    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fbresult::Waypoint>>>
    MakeWaypoints(flatbuffers::FlatBufferBuilder &fb_result,
                  const std::vector<std::vector<NodeID>> &sub_trips,
                  const std::vector<PhantomNodeCandidates> &candidates) const
    {
        std::vector<flatbuffers::Offset<fbresult::Waypoint>> waypoints;
        waypoints.reserve(parameters.coordinates.size());

        auto input_idx_to_trip_idx = MakeTripIndices(sub_trips);

        for (auto input_index : util::irange<std::size_t>(0UL, parameters.coordinates.size()))
        {
            auto trip_index = input_idx_to_trip_idx[input_index];
            BOOST_ASSERT(!trip_index.NotUsed());

            auto waypoint = BaseAPI::MakeWaypoint(&fb_result, candidates[input_index]);
            waypoint->add_waypoint_index(trip_index.point_index);
            waypoint->add_trips_index(trip_index.sub_trip_index);
            waypoints.push_back(waypoint->Finish());
        }

        return fb_result.CreateVector(waypoints);
    }

    util::json::Array MakeWaypoints(const std::vector<std::vector<NodeID>> &sub_trips,
                                    const std::vector<PhantomNodeCandidates> &candidates) const
    {
        util::json::Array waypoints;
        waypoints.values.reserve(parameters.coordinates.size());

        auto input_idx_to_trip_idx = MakeTripIndices(sub_trips);

        for (auto input_index : util::irange<std::size_t>(0UL, parameters.coordinates.size()))
        {
            auto trip_index = input_idx_to_trip_idx[input_index];
            BOOST_ASSERT(!trip_index.NotUsed());

            auto waypoint = BaseAPI::MakeWaypoint(candidates[input_index]);
            waypoint.values.emplace("trips_index", trip_index.sub_trip_index);
            waypoint.values.emplace("waypoint_index", trip_index.point_index);
            waypoints.values.push_back(std::move(waypoint));
        }

        return waypoints;
    }

    std::vector<TripIndex> MakeTripIndices(const std::vector<std::vector<NodeID>> &sub_trips) const
    {
        std::vector<TripIndex> input_idx_to_trip_idx(parameters.coordinates.size());
        for (auto sub_trip_index : util::irange<unsigned>(0u, sub_trips.size()))
        {
            for (auto point_index : util::irange<unsigned>(0u, sub_trips[sub_trip_index].size()))
            {
                input_idx_to_trip_idx[sub_trips[sub_trip_index][point_index]] =
                    TripIndex{sub_trip_index, point_index};
            }
        }
        return input_idx_to_trip_idx;
    }

    const TripParameters &parameters;
};

} // namespace osrm::engine::api

#endif
