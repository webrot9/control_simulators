#include <control_simulators/interfaces/simulable.h>
#include <control_simulators/pendulum.h>
#include <control_simulators/double_pendulum.h>
#include <control_simulators/cartpole.h>
#include <control_simulators/bicycle.h>

#include <python/functor_signature.h> // note: has to be before boost/python.hpp
#include <boost/python.hpp>
#include <boost/numpy.hpp>
//#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <Eigen/Dense>



namespace bp = boost::python;
namespace np = boost::numpy;

Eigen::MatrixXd stdvec_of_states_to_mat(const std::vector<ConsistentVector> &vec, int state_dim) {
    size_t num_states = vec.size();
    Eigen::MatrixXd all_states(num_states, state_dim);
    for (size_t i = 0; i < num_states; ++i) {
        all_states.row(i) = vec[i];
    }
    return all_states;
}

// Copies Elements from an std::vector into a new Python List
template<class T> struct VectorToPyList {
    static PyObject* convert(const std::vector<T> &vec)
    {
        bp::list py_list;
        for (const auto &item : vec) {
            py_list.append(item);
        }
        return bp::incref(bp::object(py_list).ptr());
    }
};

BOOST_PYTHON_MODULE(python_simulable)
{
    np::initialize();
    bp::import("boost_numpy_eigen");

    //bp::class_<std::vector<std::string>>("VectorStr").def(bp::vector_indexing_suite<std::vector<std::string>>());
    bp::to_python_converter<std::vector<std::string>, VectorToPyList<std::string>>();

    bp::class_<Simulable>("Simulable", bp::init<Eigen::VectorXd>())
        .def("reset", static_cast<void(Simulable::*)(void)>(&Simulable::reset))
        .def("reset", static_cast<void(Simulable::*)(const ConsistentVector&)>(&Simulable::reset))
        .def("step", &Simulable::step)
        .def("set_noise_mean", &Simulable::setNoiseMean)
        .def("set_noise_cov", &Simulable::setNoiseCovariance)
        .def("get_noise_mean", [](const Simulable &sys) {return sys.getNoiseMean();})
        .def("get_noise_cov", [](const Simulable &sys) {return sys.getNoiseCovariance();})
        .def("state", [](Simulable &sys) {return sys.state();}) 
        .def("initial_state", [](Simulable &sys) {return ConsistentVector(sys.initialState());}) 
        .def("state_size", &Simulable::stateSize)
        .def("control_size", &Simulable::controlSize)
        .def("time", &Simulable::time)
        .def("id",  &Simulable::id)
        .def("all_states", [](Simulable &sys) {
                    return stdvec_of_states_to_mat(sys.allStates(), sys.stateSize()); 
                }
            )
        .def("all_controls", [](Simulable &sys) {
                    return stdvec_of_states_to_mat(sys.allControls(), sys.controlSize()); 
                }
            )
        .def("__getitem__", [](const Simulable &sys, const std::string &key) {return sys.getParam(key);})
        .def("__setitem__", [](Simulable &sys, const std::string &key, double value) {sys.setParam(key, value);})
        .def("params", &Simulable::paramNames)
        ;

    bp::class_<Pendulum, bp::bases<Simulable>>("Pendulum", bp::init<Eigen::VectorXd>());
    bp::class_<DoublePendulum, bp::bases<Simulable>>("DoublePendulum", bp::init<Eigen::VectorXd>());
    bp::class_<Cartpole, bp::bases<Simulable>>("Cartpole", bp::init<Eigen::VectorXd>());
    bp::class_<Bicycle, bp::bases<Simulable>>("Bicycle", bp::init<Eigen::VectorXd>());

};
