#pragma once

#include <stdexcept>
#include <string>
#include "types/TrajectoryPoint.h"
#include "types/Pose2D.h"
#include "types/Event.h"
#include "simdjson/simdjson.h"
#include "utils/MathUtils.h"
#include "utils/AngleUtils.h"
#include "paths.hpp"

#include <fstream>

class Trajectory
{
public:
    Trajectory(std::string pathName)
    {
        parseFile(pathName);
    }

    Trajectory() = default;

    void parseFile(std::string pathName)
    {
        auto path = AUTO_TRAJECTORIES.find(pathName);
        if (path == AUTO_TRAJECTORIES.end())
        {
            throw std::runtime_error("Trajectory not found: " + pathName);
        }

        simdjson::ondemand::parser parser;
        simdjson::padded_string json_str(path->second);
        simdjson::ondemand::document doc = parser.iterate(json_str);

        auto samples = doc["trajectory"]["samples"];

        if (samples.error() != simdjson::SUCCESS)
        {
            throw std::runtime_error("Failed to parse trajectory file: " + pathName);
        }

        // trajectory points
        for (auto sample : samples)
        {
            TrajectoryPoint point;

            point.t = double(sample["t"]);

            point.pose.x = MathUtils::metersToInches(double(sample["x"]));
            point.pose.y = MathUtils::metersToInches(double(sample["y"]));
            point.pose.heading = double(sample["heading"]);

            point.velocity.x = MathUtils::metersToInches(double(sample["vx"]));
            point.velocity.y = MathUtils::metersToInches(double(sample["vy"]));
            point.velocity.heading = double(sample["omega"]);

            addPoint(point);
        }

        // event markers
        auto event_markers = doc["events"];
        if (event_markers.error() == simdjson::SUCCESS)
        {
            for (auto event : event_markers)
            {
                std::string_view type_view = event["event"]["type"];
                std::string type = std::string(type_view);

                // verify that it's a named event marker
                if (type != "named")
                {
                    throw std::runtime_error("Unsupported event marker type " + type + " in trajectory: " + pathName + ". Only named event markers are supported.");
                }

                std::string_view name_view = event["event"]["data"]["name"];
                std::string name = std::string(name_view);
                double time = double(event["from"]["targetTimestamp"]);

                Event e{name, time};
                eventMarkers.emplace_back(e);
            }
        }
        else
        {
            throw std::runtime_error("Failed to parse event markers from trajectory: " + pathName);
        }

        // sort event markers by time in case they aren't already sorted
        std::sort(eventMarkers.begin(), eventMarkers.end(), [](const Event &a, const Event &b)
                  { return a.time < b.time; });

        // parse splits (indices where the robot comes to a full stop)
        auto splits_node = doc["trajectory"]["splits"];
        if (splits_node.error() == simdjson::SUCCESS)
        {
            for (auto split : splits_node)
            {
                splitIndices.push_back(int(double(split)));
            }
        }

        loaded = true;
    }

    void addPoint(TrajectoryPoint point)
    {
        points.emplace_back(point);
    }

    const std::vector<TrajectoryPoint> &getPoints() const
    {
        return points;
    }

    const TrajectoryPoint &getStart() const
    {
        if (points.empty())
        {
            throw std::runtime_error("Trajectory has no points");
        }
        return points.front();
    }

    const std::vector<Event> &getEvents() const
    {
        return eventMarkers;
    }

    /**
     * @brief Splits a continuous Choreo trajectory into sub-trajectories based on Stop Points.
     * @return A vector of independent Trajectory objects, each starting at t=0.
     */
    std::vector<Trajectory> getSplits()
    {
        std::vector<Trajectory> result;

        // if there are no splits or only one split at the end, return the whole trajectory
        if (splitIndices.size() < 2 || points.empty())
        {
            result.push_back(*this);
            return result;
        }

        // add a split at the end if there isn't one already to ensure the last segment is included
        if (splitIndices.back() != points.size() - 1)
        {
            splitIndices.push_back(points.size() - 1);
        }

        for (size_t i = 0; i < splitIndices.size() - 1; ++i)
        {
            Trajectory part;
            int startIdx = splitIndices[i];
            int endIdx = splitIndices[i + 1];

            // Ensure indices are within bounds
            if (startIdx < 0 || endIdx >= points.size() || startIdx >= endIdx)
            {
                continue;
            }

            double startTime = points[startIdx].t;

            // 1. Copy and shift trajectory points
            for (int j = startIdx; j <= endIdx; ++j)
            {
                TrajectoryPoint p = points[j];
                p.t -= startTime; // Shift time so this sub-path starts at t=0
                part.addPoint(p);
            }

            // 2. Copy and shift events that fall within this sub-path
            for (const auto &e : eventMarkers)
            {
                if (e.time >= points[startIdx].t && e.time <= points[endIdx].t)
                {
                    Event shiftedEvent = e;
                    shiftedEvent.time -= startTime;
                    part.eventMarkers.push_back(shiftedEvent);
                }
            }

            part.loaded = true;
            result.push_back(part);
        }

        return result;
    }

    void clear()
    {
        points.clear();
        eventMarkers.clear();
        splitIndices.clear();
    }

    bool isLoaded() const
    {
        return loaded;
    }

private:
    std::vector<TrajectoryPoint> points;
    std::vector<Event> eventMarkers;
    std::vector<int> splitIndices;
    bool loaded = false;
};