#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "Vector2.h"
#include "Agent.h" 
#include "RVOSimulator.h"

namespace py = pybind11;

using RVO = RVO_SPACE::RVOSimulator; 
using Vector2 = RVO_SPACE::Vector2;
using AgentData = RVO_SPACE::AgentData;

PYBIND11_MODULE(rvo2, m) {
     m.doc() = "Classical ORCA - rvo2 implementation";
    
     py::class_<Vector2>(m, "Vector2")
          .def(py::init<float, float>())
          .def_property_readonly("x", &Vector2::x)
          .def_property_readonly("y", &Vector2::y);

     py::class_<RVO>(m, "RVOSimulator")
          .def(py::init<>())
          .def(py::init<float, float, size_t, float, float, float, float, const Vector2 &>(),
               py::arg("timeStep"), py::arg("neighborDist"), py::arg("maxNeighbors"),
               py::arg("timeHorizon"), py::arg("timeHorizonObst"), py::arg("radius"),
               py::arg("maxSpeed"), py::arg("velocity") = Vector2())

          .def("doStep", &RVO::doStep, "Perform a simulation step (computeNeighbors + computeNewVelocities)")
          .def("computeNeighbors", &RVO::computeNeighbors,
               "First half of a step: compute neighbors and reset time horizons to default")
          .def("computeNewVelocities", &RVO::computeNewVelocities,
               "Second half of a step: compute new velocities and update agents")

          // Agents Management
          .def("addAgent", py::overload_cast<const Vector2 &>(&RVO::addAgent),
               py::arg("position"), "Add agent with default properties")

          .def("addAgent", py::overload_cast<const Vector2 &, float, size_t, float, float, float, float, const Vector2 &>(&RVO::addAgent),
               py::arg("position"), py::arg("neighborDist"), py::arg("maxNeighbors"),
               py::arg("timeHorizon"), py::arg("timeHorizonObst"), py::arg("radius"),
               py::arg("maxSpeed"), py::arg("velocity") = Vector2(),
               "Add agent with custom properties")

          .def("addObstacle", &RVO::addObstacle, "Add a sequence of vertices, an obstacle can be an open polyline.")
          .def("addObstacle", [](RVO &sim, py::array_t<float, py::array::c_style | py::array::forcecast> vertices) {
                    if (vertices.ndim() != 2 || vertices.shape(1) != 2)
                         throw std::invalid_argument("addObstacle: expected an array of shape (N, 2)");
                    auto v = vertices.unchecked<2>();
                    std::vector<Vector2> points;
                    points.reserve(v.shape(0));
                    for (py::ssize_t i = 0; i < v.shape(0); ++i)
                         points.emplace_back(v(i, 0), v(i, 1));
                    return sim.addObstacle(points);
               }, py::arg("vertices"), "Add an obstacle from a numpy array of shape (N, 2).")
          .def("processObstacles", &RVO::processObstacles,
               "Build the obstacle kd-tree. Must be called after adding obstacles, otherwise they are ignored.")

          // Getters
          .def("getAgentPosition", &RVO::getAgentPosition, py::return_value_policy::reference)
          .def("getAgentVelocity", &RVO::getAgentVelocity, py::return_value_policy::reference)
          .def("getAgentPrefVelocity", &RVO::getAgentPrefVelocity, py::return_value_policy::reference)
          .def("getAgentNumAgentNeighbors", &RVO::getAgentNumAgentNeighbors)
          .def("getAgentAgentNeighbor", &RVO::getAgentAgentNeighbor)
          .def("getAgentRadius", &RVO::getAgentRadius)
          .def("getAgentTimeHorizon", &RVO::getAgentTimeHorizon)
          .def("getAgentTimeHorizonObst", &RVO::getAgentTimeHorizonObst)
          .def("getAgentActiveFlag", &RVO::getAgentActiveFlag)
          .def("getAgentIsArrivedFlag", &RVO::getAgentIsArrivedFlag)
          .def("getAgentORCALine", &RVO::getAgentORCALine, py::return_value_policy::reference)
          .def("getGlobalTime", &RVO::getGlobalTime)
          .def("getNumAgents", &RVO::getNumAgents)
          .def("getTimeStep", &RVO::getTimeStep)
          .def("getAgentData", &RVO::getAgentData) 
          .def("isColliding", &RVO::isColliding)
          .def("getState", &RVO::getState)
          .def("getAgentNumAgentNeighbors", &RVO::getAgentNumAgentNeighbors)

          // Setters
          .def("setAgentDefaults", &RVO::setAgentDefaults,
               py::arg("neighborDist"), py::arg("maxNeighbors"), py::arg("timeHorizon"),
               py::arg("timeHorizonObst"), py::arg("radius"), py::arg("maxSpeed"),
               py::arg("velocity") = Vector2(), "Assigns a navigation network to the simulator")
          .def("setAgentMaxNeighbors", &RVO::setAgentMaxNeighbors)
          .def("setAgentMaxSpeed", &RVO::setAgentMaxSpeed)
          .def("setAgentNeighborDist", &RVO::setAgentNeighborDist)
          .def("setAgentPosition", &RVO::setAgentPosition)
          .def("setAgentPrefVelocity", &RVO::setAgentPrefVelocity)
          .def("setAgentRadius", &RVO::setAgentRadius)
          .def("setAgentTimeHorizon", &RVO::setAgentTimeHorizon)
          .def("setAgentTimeHorizonObst", &RVO::setAgentTimeHorizonObst)
          .def("setAgentVelocity", &RVO::setAgentVelocity)
          .def("setAgentActiveFlag", &RVO::setAgentActiveFlag)
          .def("setAgentIsArrivedFlag", &RVO::setAgentIsArrivedFlag)
          .def("setTimeStep", &RVO::setTimeStep);

     py::class_<RVO_SPACE::Line>(m, "Line")
          .def_readwrite("point", &RVO_SPACE::Line::point)
          .def_readwrite("direction", &RVO_SPACE::Line::direction);

     py::class_<AgentData>(m, "AgentData")
          .def_readonly("neighbors", &AgentData::neighbors)
          .def_readonly("n_lines", &AgentData::n_lines)
          .def_readonly("n_restrictive", &AgentData::n_restrictive)
          .def_readonly("n_active", &AgentData::n_active)
          .def_readonly("n_collision_lines", &AgentData::n_collision_lines)
          .def_readonly("infeasible", &AgentData::infeasible)
          .def_readonly("neighbors_saturated", &AgentData::neighbors_saturated)
          .def_readonly("speed_clamp", &AgentData::speed_clamp)
          .def_readonly("violation", &AgentData::violation);

}