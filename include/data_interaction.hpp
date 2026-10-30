# ifndef __DATA_INTERACTION__
# define __DATA_INTERACTION__

# include <array>


# include <opencv2/core.hpp>
# include <opencv2/core/types.hpp>
# include <eigen3/Eigen/Eigen>


namespace pnp {
struct PnPResult
{
    bool valid = false;
    Eigen::Matrix<double, 4, 1> measurement {};
};





class pnpSolver
{
    public:
    //构造函数（未写）

    void easySolver();

    private:
    Eigen::Matrix<double,4,1> output {};

};

}//namespace pnp
# endif