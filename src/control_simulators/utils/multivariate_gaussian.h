#ifndef _MULTIVARIATE_GAUSSIAN_H_
#define _MULTIVARIATE_GAUSSIAN_H_

// Created by Humphrey Hu (humphreh@cs.cmu.edu)

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

#include <boost/random/random_device.hpp>
#include <boost/random/random_number_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <cassert>

/*! \brief Simple multivariate normal sampling and PDF class. */

template <typename Scalar>
class MultivariateGaussian 
{
public:
    typedef boost::mt19937 Engine; 

	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorType;
	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixType;

	typedef boost::normal_distribution<Scalar> UnivariateNormal;
	typedef boost::variate_generator<Engine&, UnivariateNormal> RandAdapter;

	/*! \brief Constructor that only requires dimension; seeds the engine using a true random number. */
	MultivariateGaussian(int dim) : 
        MultivariateGaussian(VectorType::Zero(dim, 1), MatrixType::Zero(dim, dim)) {};
	/*! \brief Constructor that only requires dimension; seeds the engine using a specified seed. */
	MultivariateGaussian(int dim, unsigned long seed) : 
        MultivariateGaussian(VectorType::Zero(dim, 1), MatrixType::Zero(dim, dim), seed) {};
	
	/*! \brief Seeds the engine using a true random number. */
	MultivariateGaussian( const VectorType& u, const MatrixType& S ) 
        : distribution_( 0.0, 1.0 ), adapter_( generator_, distribution_ ), 
        mean_( u ), covariance_( S ) 
    {
		boost::random::random_device rng;
		generator_.seed( rng );
		initialize();
	}
    
	/*! \brief Seeds the engine using a specified seed. */
	MultivariateGaussian( const VectorType& u, const MatrixType& S, unsigned long seed ) 
        : distribution_( 0.0, 1.0 ), adapter_( generator_, distribution_ ), 
        mean_( u ), covariance_( S ) 
    {
		generator_.seed( seed );
		initialize();
	}
    
    void setMean( const VectorType& u ) { mean_ = VectorType(u); }
    void setCovariance( const MatrixType& S ) {
        covariance_ = MatrixType(S);
        initialize();
    }
    
	const VectorType& getMean() const { return mean_; }
	const MatrixType& getCovariance() const { return covariance_; }
	const MatrixType& getCholesky() const { return L_; }
	
	/*! \brief Generate a sample truncated at a specified number of standard deviations. */
	VectorType sample( double v = 3.0 ) {
        const int dim = mean_.size();
		VectorType samples(dim);
		for(int i = 0; i < dim; ++i ) {
			double s;
			do {
				s = adapter_();
			} while( std::abs( s ) > v );
			samples(i) = s;
		}
		
		return mean_ + L_*samples;
	}

	/*! \brief Evaluate the multivariate normal PDF for the specified sample. */
    double evaluateProbability( const VectorType& x ) const {
		const VectorType diff = x - mean_;
		const double exponent = -0.5 * diff.dot( llt_decomp_.solve( diff ) );
		return Z_ * std::exp( exponent );
	}
    
protected:
	
	Engine generator_;
	UnivariateNormal distribution_;
	RandAdapter adapter_;

	VectorType mean_;
	MatrixType covariance_;

	MatrixType L_;
	Eigen::LLT<MatrixType> llt_decomp_;

	double Z_; // Normalization constant;
	
	void initialize()
	{
		assert( mean_.rows() == covariance_.rows() );
		assert( mean_.rows() == covariance_.cols() );
		llt_decomp_ = Eigen::LLT<MatrixType>( covariance_ );
		L_ = llt_decomp_.matrixL();
        const double det = covariance_.determinant();
		Z_ = std::pow( 2*M_PI, -mean_.size()/2.0 ) * std::pow(det , -0.5 );
	}

};

#endif
