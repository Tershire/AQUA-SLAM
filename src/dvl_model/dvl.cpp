#include <iostream>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <waterlinked_a50_ros_driver/DVL.h>
#include <waterlinked_a50_ros_driver/DVLBeam.h>
#include <Thirdparty/g2o/g2o/core/base_binary_edge.h>
#include <Thirdparty/g2o/g2o/core/sparse_block_matrix.h>
#include <Thirdparty/g2o/g2o/core/block_solver.h>
#include <Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h>
#include <Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h>
#include <Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h>
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <Thirdparty/g2o/g2o/core/robust_kernel_impl.h>
#include <Thirdparty/g2o/g2o/solvers/linear_solver_dense.h>
#include <Thirdparty/g2o/g2o/types/se3mat.h>
#include <mat.h>

class DvlData
{
public:
	DvlData()
		: mVelocityFromBeam(Eigen::Vector3d::Identity()), mVelocity(Eigen::Vector3d::Identity()), mAlpha(0), mBeta(0),
		  mE(Eigen::Matrix<double, 4, 3>::Identity()), mVelocityBeam(Eigen::Vector4d::Identity())
	{

	}
	DvlData(const waterlinked_a50_ros_driver::DVL &dvl, double alpha, double beta)
		: mAlpha(alpha), mBeta(beta)
	{
		mVelocity.x() = dvl.velocity.x;
		mVelocity.y() = dvl.velocity.y;
		mVelocity.z() = dvl.velocity.z;

		mE << -cos(beta) * cos(alpha), sin(beta) * cos(alpha), sin(alpha),
			-cos(beta) * cos(alpha), -sin(beta) * cos(alpha), sin(alpha),
			cos(beta) * cos(alpha), -sin(beta) * cos(alpha), sin(alpha),
			cos(beta) * cos(alpha), sin(beta) * cos(alpha), sin(alpha);

		for (auto b: dvl.beams) {
			std::pair<int, double> beam(b.id, b.velocity);
			mBeams.insert(beam);
			mVelocityBeam(b.id) = b.velocity;
		}
		mVelocityFromBeam = (mE.transpose() * mE).inverse() * mE.transpose() * mVelocityBeam;
	}
	DvlData(const DvlData &d)
		: mVelocityFromBeam(d.mVelocityFromBeam), mAlpha(d.mAlpha), mBeta(d.mBeta), mE(d.mE), mVelocity(d.mVelocity),
		  mBeams(d.mBeams), mVelocityBeam(d.mVelocityBeam)
	{

	}
	double mAlpha, mBeta;
	Eigen::Matrix<double, 4, 3> mE;
	Eigen::Vector3d mVelocity, mVelocityFromBeam;
	Eigen::Vector4d mVelocityBeam;
	std::map<int, double> mBeams;
};

class VertexDVLBeamOritenstion: public g2o::BaseVertex<2, Eigen::Vector2d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexDVLBeamOritenstion()
	{}
	VertexDVLBeamOritenstion(const Eigen::Vector2d &r)
	{
		setEstimate(Eigen::Vector2d(r));
	}

	virtual bool read(std::istream &is)
	{}
	virtual bool write(std::ostream &os) const
	{}

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		_estimate.x() += update_[0];
		_estimate.y() += update_[1];
		updateCache();
	}
};

class VertexVelocity: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexVelocity()
	{}
	VertexVelocity(const Eigen::Vector3d &r)
	{
		setEstimate(Eigen::Vector3d(r));
	}

	virtual bool read(std::istream &is)
	{}
	virtual bool write(std::ostream &os) const
	{}

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		_estimate.x() += update_[0];
		_estimate.y() += update_[1];
		_estimate.z() += update_[2];
		updateCache();
	}
};

class VertexDVLBeamSphericalOritenstion: public g2o::BaseVertex<8, Eigen::Matrix<double, 8, 1>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexDVLBeamSphericalOritenstion()
	{}
	VertexDVLBeamSphericalOritenstion(const Eigen::Matrix<double, 8, 1> &r)
	{
		setEstimate(Eigen::Matrix<double, 8, 1>(r));
	}

	virtual bool read(std::istream &is)
	{}
	virtual bool write(std::ostream &os) const
	{}

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		_estimate(0) += update_[0];
		_estimate(1) += update_[1];
		_estimate(2) += update_[2];
		_estimate(3) += update_[3];
		_estimate(4) += update_[4];
		_estimate(5) += update_[5];
		_estimate(6) += update_[6];
		_estimate(7) += update_[7];
		updateCache();
	}
};

class EdgeDVLBeamCalibration: public g2o::BaseUnaryEdge<3, DvlData, VertexDVLBeamOritenstion>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDVLBeamCalibration()
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexDVLBeamOritenstion *V = static_cast<const VertexDVLBeamOritenstion *>(_vertices[0]);
		const Eigen::Vector3d velocity_ob(_measurement.mVelocity);
		//r[0] alpha in paper, rotation from horizental plane
		//r[1] beta in paper, rotation arround z-axis
		Eigen::Vector2d r = V->estimate();

		Eigen::Vector3d velocity_est(0, 0, 0);

		for (int id = 0; id < 4; id++) {
			if (id == 0) {
				Eigen::Vector3d v_e(-sin(r(1)) * cos(r(0)) * _measurement.mBeams[id],
				                    cos(r(1)) * cos(r(0)) * _measurement.mBeams[id],
				                    sin(r(1)) * _measurement.mBeams[id]);
				velocity_est = velocity_est + v_e;
			}
			else if (id == 1) {
				Eigen::Vector3d v_e(-cos(r(1)) * cos(r(0)) * _measurement.mBeams[id],
				                    -sin(r(1)) * cos(r(0)) * _measurement.mBeams[id],
				                    sin(r(1)) * _measurement.mBeams[id]);
				velocity_est = velocity_est + v_e;

			}
			else if (id == 2) {
				Eigen::Vector3d v_e(sin(r(1)) * cos(r(0)) * _measurement.mBeams[id],
				                    -cos(r(1)) * cos(r(0)) * _measurement.mBeams[id],
				                    sin(r(1)) * _measurement.mBeams[id]);
				velocity_est = velocity_est + v_e;

			}
			else if (id == 3) {
				Eigen::Vector3d v_e(cos(r(1)) * cos(r(0)) * _measurement.mBeams[id],
				                    sin(r(1)) * cos(r(0)) * _measurement.mBeams[id],
				                    sin(r(1)) * _measurement.mBeams[id]);
				velocity_est = velocity_est + v_e;

			}
		}

//		for (int id = 0; id < 4; id++) {
//			if (id == 0) {
//				Eigen::Vector3d v_e(-sin(45/180.0 * M_PI) * cos(r(0)) * _measurement.mBeams[id],
//				                    cos(45/180.0 * M_PI) * cos(r(0)) * _measurement.mBeams[id],
//				                    sin(45/180.0 * M_PI) * _measurement.mBeams[id]);
//				velocity_est.push_back(v_e);
//			}
//			else if (id == 1) {
//				Eigen::Vector3d v_e(-cos(45/180.0 * M_PI) * cos(r(0)) * _measurement.mBeams[id],
//				                    -sin(45/180.0 * M_PI) * cos(r(0)) * _measurement.mBeams[id],
//				                    sin(45/180.0 * M_PI) * _measurement.mBeams[id]);
//				velocity_est.push_back(v_e);
//
//			}
//			else if (id == 2) {
//				Eigen::Vector3d v_e(sin(45/180.0 * M_PI) * cos(r(0)) * _measurement.mBeams[id],
//				                    -cos(45/180.0 * M_PI) * cos(r(0)) * _measurement.mBeams[id],
//				                    sin(45/180.0 * M_PI) * _measurement.mBeams[id]);
//				velocity_est.push_back(v_e);
//
//			}
//			else if (id == 3) {
//				Eigen::Vector3d v_e(cos(45/180.0 * M_PI) * cos(r(0)) * _measurement.mBeams[id],
//				                    sin(45/180.0 * M_PI) * cos(r(0)) * _measurement.mBeams[id],
//				                    sin(45/180.0 * M_PI) * _measurement.mBeams[id]);
//				velocity_est.push_back(v_e);
//
//			}
//		}

		_error << velocity_est - velocity_ob;
//		_error << Eigen::Matrix<double,12,1>::Identity();
	}

//	virtual void linearizeOplus();

//	Eigen::Matrix<double, 12, 12> GetHessian()
//	{
//		linearizeOplus();
//		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
//	}

public:

};

class EdgeDVLBeamCalibration2: public g2o::BaseUnaryEdge<4, DvlData, VertexDVLBeamOritenstion>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDVLBeamCalibration2()
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexDVLBeamOritenstion *V = static_cast<const VertexDVLBeamOritenstion *>(_vertices[0]);
		const Eigen::Vector3d velocity_est(_measurement.mVelocity);
		//r[0] alpha in paper, rotation from horizental plane
		//r[1] beta in paper, rotation arround z-axis
		Eigen::Vector2d r = V->estimate();
		std::vector<double> v_beam_est;

		Eigen::Vector4d err(0, 0, 0, 0);
//		r(1) = 45.0 / 180 * M_PI;

		for (int id = 0; id < 4; id++) {
			if (id == 0) {
				double v_beam = -velocity_est.x() * cos(r(1)) * cos(r(0)) + velocity_est.y() * sin(r(1)) * cos(r(0))
					+ sin(r(0)) * velocity_est.z();
				double e = _measurement.mBeams[id] - v_beam;
				err.x() = e;
			}
			else if (id == 1) {
				double v_beam = -velocity_est.x() * cos(r(1)) * cos(r(0)) - velocity_est.y() * sin(r(1)) * cos(r(0))
					+ sin(r(0)) * velocity_est.z();
				double e = _measurement.mBeams[id] - v_beam;
				err.y() = e;

			}
			else if (id == 2) {
				double v_beam = velocity_est.x() * cos(r(1)) * cos(r(0)) - velocity_est.y() * sin(r(1)) * cos(r(0))
					+ sin(r(0)) * velocity_est.z();
				double e = _measurement.mBeams[id] - v_beam;
				err.z() = e;

			}
			else if (id == 3) {
				double v_beam = velocity_est.x() * cos(r(1)) * cos(r(0)) + velocity_est.y() * sin(r(1)) * cos(r(0))
					+ sin(r(0)) * velocity_est.z();
				double e = _measurement.mBeams[id] - v_beam;
				err.w() = e;
			}
		}


		_error << err;
//		_error << Eigen::Matrix<double,12,1>::Identity();
	}

//	virtual void linearizeOplus();

//	Eigen::Matrix<double, 12, 12> GetHessian()
//	{
//		linearizeOplus();
//		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
//	}

public:

};

class EdgeDVLBeamCalibration3: public g2o::BaseUnaryEdge<4, DvlData, VertexDVLBeamSphericalOritenstion>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDVLBeamCalibration3()
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexDVLBeamSphericalOritenstion
			*V = static_cast<const VertexDVLBeamSphericalOritenstion *>(_vertices[0]);
		const Eigen::Vector3d velocity_est(_measurement.mVelocity);
		//r[0] alpha in paper, rotation from horizental plane
		//r[1] beta in paper, rotation arround z-axis
		Eigen::Matrix<double, 8, 1> r = V->estimate();
		std::vector<double> v_beam_est;

		Eigen::Vector4d err(0, 0, 0, 0);
		r(1) = (45.0 + 90) / 180 * M_PI;
		r(3) = (45.0 + 180) / 180 * M_PI;
		r(5) = (45.0 + 270) / 180 * M_PI;
		r(7) = 45.0 / 180 * M_PI;

//		r(0) = 32.5 / 180 * M_PI;
		r(2) = 32.5 / 180 * M_PI;
		r(4) = 32.5 / 180 * M_PI;
		r(6) = 32.5 / 180 * M_PI;

		for (int id = 0; id < 4; id++) {
			if (id == 0) {
				Eigen::Vector3d v_dvl_beam(sin(r(0)) * cos(r(1)) * _measurement.mBeams[id],
				                           sin(r(0)) * sin(r(1)) * _measurement.mBeams[id],
				                           cos(r(0)) * _measurement.mBeams[id]);
				Eigen::Vector3d v_dvl_beam_normalized = v_dvl_beam.normalized();
				Eigen::Vector3d v_x = Eigen::Vector3d::UnitX() * _measurement.mVelocity.x();
				Eigen::Vector3d v_y = Eigen::Vector3d::UnitY() * _measurement.mVelocity.y();
				Eigen::Vector3d v_z = Eigen::Vector3d::UnitZ() * _measurement.mVelocity.z();
				double v_beam = (v_x.transpose() * v_dvl_beam_normalized + v_y.transpose() * v_dvl_beam_normalized
					+ v_z.transpose() * v_dvl_beam_normalized).x();
				double e = _measurement.mBeams[id] - v_beam;
				err.x() = e;
			}
			else if (id == 1) {
				Eigen::Vector3d v_dvl_beam(sin(r(2)) * cos(r(3)) * _measurement.mBeams[id],
				                           sin(r(2)) * sin(r(3)) * _measurement.mBeams[id],
				                           cos(r(2)) * _measurement.mBeams[id]);
				Eigen::Vector3d v_dvl_beam_normalized = v_dvl_beam.normalized();
				Eigen::Vector3d v_x = Eigen::Vector3d::UnitX() * _measurement.mVelocity.x();
				Eigen::Vector3d v_y = Eigen::Vector3d::UnitY() * _measurement.mVelocity.y();
				Eigen::Vector3d v_z = Eigen::Vector3d::UnitZ() * _measurement.mVelocity.z();
				double v_beam = (v_x.transpose() * v_dvl_beam_normalized + v_y.transpose() * v_dvl_beam_normalized
					+ v_z.transpose() * v_dvl_beam_normalized).x();
				double e = _measurement.mBeams[id] - v_beam;
				err.y() = e;

			}
			else if (id == 2) {
				Eigen::Vector3d v_dvl_beam(sin(r(4)) * cos(r(5)) * _measurement.mBeams[id],
				                           sin(r(4)) * sin(r(5)) * _measurement.mBeams[id],
				                           cos(r(4)) * _measurement.mBeams[id]);
				Eigen::Vector3d v_dvl_beam_normalized = v_dvl_beam.normalized();
				Eigen::Vector3d v_x = Eigen::Vector3d::UnitX() * _measurement.mVelocity.x();
				Eigen::Vector3d v_y = Eigen::Vector3d::UnitY() * _measurement.mVelocity.y();
				Eigen::Vector3d v_z = Eigen::Vector3d::UnitZ() * _measurement.mVelocity.z();
				double v_beam = (v_x.transpose() * v_dvl_beam_normalized + v_y.transpose() * v_dvl_beam_normalized
					+ v_z.transpose() * v_dvl_beam_normalized).x();
				double e = _measurement.mBeams[id] - v_beam;
				err.z() = e;

			}
			else if (id == 3) {
				Eigen::Vector3d v_dvl_beam(sin(r(6)) * cos(r(7)) * _measurement.mBeams[id],
				                           sin(r(6)) * sin(r(7)) * _measurement.mBeams[id],
				                           cos(r(6)) * _measurement.mBeams[id]);
				Eigen::Vector3d v_dvl_beam_normalized = v_dvl_beam.normalized();
				Eigen::Vector3d v_x = Eigen::Vector3d::UnitX() * _measurement.mVelocity.x();
				Eigen::Vector3d v_y = Eigen::Vector3d::UnitY() * _measurement.mVelocity.y();
				Eigen::Vector3d v_z = Eigen::Vector3d::UnitZ() * _measurement.mVelocity.z();
				double v_beam = (v_x.transpose() * v_dvl_beam_normalized + v_y.transpose() * v_dvl_beam_normalized
					+ v_z.transpose() * v_dvl_beam_normalized).x();
				double e = _measurement.mBeams[id] - v_beam;
				err.w() = e;
			}
		}


		_error << err;
//		_error << Eigen::Matrix<double,12,1>::Identity();
	}

//	virtual void linearizeOplus();

//	Eigen::Matrix<double, 12, 12> GetHessian()
//	{
//		linearizeOplus();
//		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
//	}

public:

};

class EdgeDVLBeamCalibration4: public g2o::BaseUnaryEdge<4, DvlData, VertexDVLBeamSphericalOritenstion>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDVLBeamCalibration4()
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexDVLBeamSphericalOritenstion
			*V = static_cast<const VertexDVLBeamSphericalOritenstion *>(_vertices[0]);
		const Eigen::Vector3d velocity_est(_measurement.mVelocity);
		//r[0] alpha in paper, rotation from horizental plane
		//r[1] beta in paper, rotation arround z-axis
		Eigen::Matrix<double, 8, 1> r = V->estimate();
		std::vector<double> v_beam_est;

		Eigen::Vector4d err(0, 0, 0, 0);
//		r(1) = 45.0 / 180 * M_PI;

		for (int id = 0; id < 4; id++) {
			if (id == 0) {
				double v_beam = -velocity_est.x() * cos(r(1)) * cos(r(0)) + velocity_est.y() * sin(r(1)) * cos(r(0))
					+ sin(r(0)) * velocity_est.z();
				double e = _measurement.mBeams[id] - v_beam;
				err.x() = e;
			}
			else if (id == 1) {
				double v_beam = -velocity_est.x() * cos(r(3)) * cos(r(2)) - velocity_est.y() * sin(r(3)) * cos(r(2))
					+ sin(r(2)) * velocity_est.z();
				double e = _measurement.mBeams[id] - v_beam;
				err.y() = e;

			}
			else if (id == 2) {
				double v_beam = velocity_est.x() * cos(r(5)) * cos(r(4)) - velocity_est.y() * sin(r(5)) * cos(r(4))
					+ sin(r(4)) * velocity_est.z();
				double e = _measurement.mBeams[id] - v_beam;
				err.z() = e;

			}
			else if (id == 3) {
				double v_beam = velocity_est.x() * cos(r(7)) * cos(r(6)) + velocity_est.y() * sin(r(7)) * cos(r(6))
					+ sin(r(6)) * velocity_est.z();
				double e = _measurement.mBeams[id] - v_beam;
				err.w() = e;
			}
		}


		_error << err;
//		_error << Eigen::Matrix<double,12,1>::Identity();
	}

//	virtual void linearizeOplus();

//	Eigen::Matrix<double, 12, 12> GetHessian()
//	{
//		linearizeOplus();
//		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
//	}

public:

};

std::mutex dvl_mutex;

std::vector<DvlData> all_dvl;

std::vector<double> v_x, v_y, v_z, vb_x, vb_y, vb_z;

double ALPHA = (67.5 / 180.0) * M_PI, BETA = (45 / 180.0) * M_PI;
//double ALPHA = (71.39 / 180.0) * M_PI, BETA = (65.89 / 180.0) * M_PI;

void dvl_cb(const waterlinked_a50_ros_driver::DVL &msg)
{
	DvlData d(msg, ALPHA, BETA);
	std::lock_guard<std::mutex> lock(dvl_mutex);
	all_dvl.push_back(d);
	v_x.push_back(d.mVelocity.x());
	v_y.push_back(d.mVelocity.y());
	v_z.push_back(d.mVelocity.z());
	vb_x.push_back(d.mVelocityFromBeam.x());
	vb_y.push_back(d.mVelocityFromBeam.y());
	vb_z.push_back(d.mVelocityFromBeam.z());
}

void OptimizeBeam(const ros::TimerEvent &)
{

	{
		std::lock_guard<std::mutex> lock(dvl_mutex);
		MATFile *pmat = matOpen("velocity.mat", "w");
		if (pmat) {
			//save data to local file
			mxArray *pm_v_x = mxCreateDoubleMatrix(v_x.size(), 1, mxREAL),
				*pm_v_y = mxCreateDoubleMatrix(v_y.size(), 1, mxREAL),
				*pm_v_z = mxCreateDoubleMatrix(v_z.size(), 1, mxREAL),
				*pm_vb_x = mxCreateDoubleMatrix(vb_x.size(), 1, mxREAL),
				*pm_vb_y = mxCreateDoubleMatrix(vb_y.size(), 1, mxREAL),
				*pm_vb_z = mxCreateDoubleMatrix(vb_z.size(), 1, mxREAL);

			memcpy((void *)(mxGetPr(pm_v_x)), (void *)v_x.data(), v_x.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_v_y)), (void *)v_y.data(), v_y.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_v_z)), (void *)v_z.data(), v_z.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_vb_x)), (void *)vb_x.data(), vb_x.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_vb_y)), (void *)vb_y.data(), vb_y.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_vb_z)), (void *)vb_z.data(), vb_z.size() * sizeof(double));
			auto status = matPutVariable(pmat, "v_x", pm_v_x);
			matPutVariable(pmat, "v_y", pm_v_y);
			matPutVariable(pmat, "v_z", pm_v_z);
			matPutVariable(pmat, "vb_x", pm_vb_x);
			matPutVariable(pmat, "vb_y", pm_vb_y);
			matPutVariable(pmat, "vb_z", pm_vb_z);
			matClose(pmat);
		}
	}


	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(true);

	// add vertex
	Eigen::Vector2d r(57.5 / 180.0 * M_PI, 45.0 / 180.0 * M_PI);
	VertexDVLBeamOritenstion *v = new VertexDVLBeamOritenstion(r);
	v->setId(0);
	v->setFixed(false);
	optimizer.addVertex(v);

	//add edge
	{
		std::lock_guard<std::mutex> lock(dvl_mutex);
		for (auto d: all_dvl) {
			EdgeDVLBeamCalibration2 *e = new EdgeDVLBeamCalibration2();
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(v));
			e->setMeasurement(d);
			e->setInformation(Eigen::Matrix<double, 4, 4>::Identity() * 180.0 / M_PI);
			optimizer.addEdge(e);
		}
	}

	optimizer.initializeOptimization();
	optimizer.optimize(6);

	Eigen::Vector2d r_opt = v->estimate();

	ROS_INFO_STREAM("DVL Calibration: alpha=" << r_opt(0) / M_PI * 180.0 << " beta=" << r_opt[1] / M_PI * 180.0);

}

void OptimizeBeamSphere(const ros::TimerEvent &)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(true);

	// add vertex
	Eigen::Matrix<double, 8, 1> r;
	r << 57.5 / 180.0 * M_PI,
		(45.0 + 90) / 180.0 * M_PI,
		57.5 / 180.0 * M_PI,
		(45.0 + 180) / 180.0 * M_PI,
		57.5 / 180.0 * M_PI,
		(45.0 + 270) / 180.0 * M_PI,
		57.5 / 180.0 * M_PI,
		45.0 / 180.0 * M_PI;
	VertexDVLBeamSphericalOritenstion *v = new VertexDVLBeamSphericalOritenstion(r);
	v->setId(0);
	v->setFixed(false);
	optimizer.addVertex(v);

	//add edge
	{
		std::lock_guard<std::mutex> lock(dvl_mutex);
		for (auto d: all_dvl) {
			EdgeDVLBeamCalibration3 *e = new EdgeDVLBeamCalibration3();
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(v));
			e->setMeasurement(d);
			e->setInformation(Eigen::Matrix<double, 4, 4>::Identity() * 180.0 / M_PI);
			optimizer.addEdge(e);
		}
	}

	optimizer.initializeOptimization();
	optimizer.optimize(6);

	Eigen::Matrix<double, 8, 1> r_opt = v->estimate();

	ROS_INFO_STREAM(
		"DVL Calibration:\nbeam1_theta=" << r_opt(0) / M_PI * 180.0 << " beam1_phi=" << r_opt(1) / M_PI * 180.0
		                                 << "\nbeam2_theta=" << r_opt(2) / M_PI * 180.0 << " beam2_phi="
		                                 << r_opt(3) / M_PI * 180.0
		                                 << "\nbeam3_theta=" << r_opt(4) / M_PI * 180.0 << " beam3_phi="
		                                 << r_opt(5) / M_PI * 180.0
		                                 << "\nbeam4_theta=" << r_opt(6) / M_PI * 180.0 << " beam4_phi="
		                                 << r_opt(7) / M_PI * 180.0);

}

void OptimizeBeam2(const ros::TimerEvent &)
{
	{
		std::lock_guard<std::mutex> lock(dvl_mutex);
		MATFile *pmat = matOpen("velocity.mat", "w");
		if (pmat) {
			//save data to local file
			mxArray *pm_v_x = mxCreateDoubleMatrix(v_x.size(), 1, mxREAL),
				*pm_v_y = mxCreateDoubleMatrix(v_y.size(), 1, mxREAL),
				*pm_v_z = mxCreateDoubleMatrix(v_z.size(), 1, mxREAL),
				*pm_vb_x = mxCreateDoubleMatrix(vb_x.size(), 1, mxREAL),
				*pm_vb_y = mxCreateDoubleMatrix(vb_y.size(), 1, mxREAL),
				*pm_vb_z = mxCreateDoubleMatrix(vb_z.size(), 1, mxREAL);

			memcpy((void *)(mxGetPr(pm_v_x)), (void *)v_x.data(), v_x.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_v_y)), (void *)v_y.data(), v_y.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_v_z)), (void *)v_z.data(), v_z.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_vb_x)), (void *)vb_x.data(), vb_x.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_vb_y)), (void *)vb_y.data(), vb_y.size() * sizeof(double));
			memcpy((void *)(mxGetPr(pm_vb_z)), (void *)vb_z.data(), vb_z.size() * sizeof(double));
			auto status = matPutVariable(pmat, "v_x", pm_v_x);
			matPutVariable(pmat, "v_y", pm_v_y);
			matPutVariable(pmat, "v_z", pm_v_z);
			matPutVariable(pmat, "vb_x", pm_vb_x);
			matPutVariable(pmat, "vb_y", pm_vb_y);
			matPutVariable(pmat, "vb_z", pm_vb_z);
			matClose(pmat);
		}
	}

	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(true);

	// add vertex
	Eigen::Matrix<double, 8, 1> r;
	r << 0 / 180.0 * M_PI,
		0 / 180.0 * M_PI,
		0 / 180.0 * M_PI,
		0 / 180.0 * M_PI,
		0 / 180.0 * M_PI,
		0 / 180.0 * M_PI,
		0 / 180.0 * M_PI,
		0 / 180.0 * M_PI;
	VertexDVLBeamSphericalOritenstion *v = new VertexDVLBeamSphericalOritenstion(r);
	v->setId(0);
	v->setFixed(false);
	optimizer.addVertex(v);

	//add edge
	{
		std::lock_guard<std::mutex> lock(dvl_mutex);
		for (auto d: all_dvl) {
			EdgeDVLBeamCalibration4 *e = new EdgeDVLBeamCalibration4();
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(v));
			e->setMeasurement(d);
			e->setInformation(Eigen::Matrix<double, 4, 4>::Identity() * 180.0 / M_PI);
			optimizer.addEdge(e);
		}
	}

	optimizer.initializeOptimization();
	optimizer.optimize(6);

	Eigen::Matrix<double, 8, 1> r_opt = v->estimate();

	ROS_INFO_STREAM(
		"DVL Calibration:\nbeam1_theta=" << r_opt(0) / M_PI * 180.0 << " beam1_phi=" << r_opt(1) / M_PI * 180.0
		                                 << "\nbeam2_theta=" << r_opt(2) / M_PI * 180.0 << " beam2_phi="
		                                 << r_opt(3) / M_PI * 180.0
		                                 << "\nbeam3_theta=" << r_opt(4) / M_PI * 180.0 << " beam3_phi="
		                                 << r_opt(5) / M_PI * 180.0
		                                 << "\nbeam4_theta=" << r_opt(6) / M_PI * 180.0 << " beam4_phi="
		                                 << r_opt(7) / M_PI * 180.0);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dvl_model");
	ros::NodeHandle n;

	ros::Subscriber dvl_sub = n.subscribe("/dvl/data", 100, dvl_cb);
//	ros::Timer opt_timer = n.createTimer(ros::Duration(3.0), OptimizeBeamSphere);
	ros::Timer opt_timer = n.createTimer(ros::Duration(3.0), OptimizeBeam2);
	ros::spin();
	return 0;
}

