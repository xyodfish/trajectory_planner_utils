#ifndef _VELOCITY_PLANNER_INTERAFACE_H
#define _VELOCITY_PLANNER_INTERAFACE_H
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace vp::tp {
    /**
     * @brief KinematicsState
     *
     * @tparam KS_T
     */
    template <typename T>
    class KState {
       public:
        T time;
        T pos;
        T vel;
        T acc;
        T jerk;
        T theta;
    };

    template <typename T>
    class BoudaryConditions {
       public:
        /* data */
        KState<T> s_state;
        KState<T> g_state;
        T max_acc;
        T max_vel;
        T max_jerk;
        T delta_t;
    };

    template <typename T>
    using BCs = BoudaryConditions<T>;

    template <typename... Args>
    class VpParamsBase {
       public:
        VpParamsBase(Args&... args) : params(std::forward<Args&>(args)...) {}
        std::tuple<Args...> params;
    };

    using Vpb3d = VpParamsBase<double, double, double>;

    class VpParams : public Vpb3d {
       public:
        VpParams(double& param1, double& param2, double& param3) : Vpb3d(param1, param2, param3) {}

       private:
        double param3_;
    };

    template <typename T>
    class VelocityPlannerInterface {
       public:
        explicit VelocityPlannerInterface() = default;
        explicit VelocityPlannerInterface(std::vector<BCs<T>> BC, const std::string& name){};
        virtual ~VelocityPlannerInterface() = default;

        /**
         * @brief Get the State object
         *
         * @param con
         * @param time
         * @return KState< T >
         */
        virtual std::vector<KState<T>> getKState(T time, bool isNormalized = false) = 0;

        /**
         * @brief Get the Traj object
         *
         * @return std::vector< KState< data_T > >
         */
        virtual std::vector<std::vector<KState<T>>> planKStates(bool isNormalized = false) = 0;

        virtual std::vector<std::vector<T>> planTrajs(bool isNormalized = false) = 0;

        virtual std::vector<T> getEndTraj(bool isNormalized = false) = 0;

        virtual std::vector<std::vector<T>> getTimeSpan() {
            std::vector<T> _span;
            for (const auto& var : trajs_) {
                _span.push_back(var[0]);
            }

            return {_span};
        }

        virtual std::vector<std::vector<T>> getPosVec() {
            std::vector<T> _pos;
            for (const auto& var : trajs_) {
                _pos.push_back(var[1]);
            }

            return {_pos};
        }

        std::vector<std::vector<T>>& getTrajs() { return trajs_; };

       protected:
        std::string alg_name;
        std::vector<std::vector<T>> trajs_;
    };

    template <typename T>
    class VelocityPlannerCompatInterface {
       public:
        explicit VelocityPlannerCompatInterface() = default;
        explicit VelocityPlannerCompatInterface(const std::vector<BCs<T>>& BCs, const std::string& algs){};
        virtual ~VelocityPlannerCompatInterface() = default;

        /**
         * @brief Get the Traj object
         *
         * @return std::vector< KState< data_T > >
         */
        virtual std::vector<std::vector<KState<T>>> getKStates(bool isNormalized = false) = 0;

        virtual std::vector<std::vector<T>> getTrajs(bool isNormalized = false) = 0;

        virtual std::vector<std::vector<T>> getPosVec() = 0;

        std::shared_ptr<VelocityPlannerInterface<T>> vp_;
    };
}  // namespace vp::tp
#endif
