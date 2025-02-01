/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */

#ifndef Antipatrea__PseudoRandom_HPP_
#define Antipatrea__PseudoRandom_HPP_

#include "Utils/Definitions.hpp"
#include <cstdlib>

namespace Antipatrea {

/**
 *@cond
 */

/**
 *@brief Operating system specific definitions 
 */
#ifdef OS_WINDOWS
#define srandom srand
#define random rand
#define RANDOM_MAX RAND_MAX
#else
#define RANDOM_MAX 2147483647
#endif

/**
 *@endcond
 */

    /**
     *@name Seed the random number generator
     *@{
     */

    /**
     *@brief Set the seed for the random number generator 
     *@param s seed value
     */
    void RandomSeed(const unsigned int s);

    /**
     *@brief Generate and set the seed for the random number generator 
     * 
     *@returns
     *  A pseudorandom number read from the file <em>/dev/urandom</em>, 
     *  which is then used to set the seed using <em>srandom</em>
     *  \n
     *  If file <em>/dev/urandom</em> does not exist, then
     *  <em>time(NULL)</em> is used to obtain the seed. 
     */
    unsigned int RandomSeed(void);

    /**
     *END Seed the random number generator
     *@}
     */

    /**
     *@name Generate real numbers uniformly at random
     *@{
     */

    /**
     *@brief Generate a real number uniformly at random from the interval
     *       <em>[0, 1]</em> 
     */
    static inline double RandomUniformReal(void) {
        return ((double) random()) / ((double) RANDOM_MAX);
    }

    /**
     *@brief Generate a real number uniformly at random from the interval
     *       <em>[min, max]</em> 
     *
     *@param min left boundary of the sampling interval
     *@param max right boundary of the sampling interval
     */
    static inline double RandomUniformReal(const double min, const double max) {
        return min + (max - min) * RandomUniformReal();
    }

    /**
     *END Generate real numbers uniformly at random
     *@}
     */

    /**
     *@name Generate integers uniformly at random
     *@{
     */

    /**
     *@brief Generate an integer number uniformly at random from the
     *       interval <em>[min, max]</em> 
     *
     *@param min left boundary of the sampling interval
     *@param max right boundary of the sampling interval
     */
    static inline long RandomUniformInteger(const long min, const long max) {
        const long x = min + (long) RandomUniformReal(0, max - min + 1);
        return x > max ? max : x;
    }

    /**
     *END Generate integer numbers uniformly at random
     *@}
     */

    /**
     *@name Generate boolean values uniformly at random
     *@{
     */

    /**
     *@brief Generate either true or false uniformly at random
     */
    static inline bool RandomUniformBoolean(void) {
        return RandomUniformReal() > 0.5;
    }

    /**
     *END Generate boolean values uniformly at random
     *@}
     */

    /**
     *@name Generate real values from gaussian distributions
     *@{
     */

    /**
     *@brief Generate a number at random from the Gaussian distribution
     *       with mean <em>mean</em> and standard deviation
     *       <em>stddev</em> 
     *
     *@param mean   mean of the Gaussian distribution.
     *@param stddev standard deviation of the Gaussian distribution. 
     *
     *@par Description:
     *  Given \f$\mu, \sigma \in R\f$, with \f$\sigma \not = 0\f$, the 
     *  \f$(\mu, \sigma^2)\f$ normal or Gaussian probability distribution
     *  function is defined by
     *  \f[
     *      P(x) = \frac{1}{\sqrt{2\pi \sigma^2}} e^{-\frac{(x - \mu)^2}{2\sigma^2}}.
     *  \f]
     *  When \f$\mu = 0\f$ and \f$\sigma^2 = 1\f$, the distribution is
     *  usually called the <em>standard normal distribution</em>.
     *  \n\n    
     *  The fundamental transformation law of probabilities allows for the
     *  design of a transformation function that takes as input random
     *  variables from one distribution and outputs random variables from
     *  another distribution. An important example of such transformation
     *  is the <em>Box-Muller</em> method for generating random samples
     *  with a Gaussian (normal) distribution using samples from a uniform
     *  distribution.
     *  \n\n 
     *  The transformation of two uniform samples <em>x1, x2</em> in
     *  <em>(0, 1)</em> into two samples <em>y1, y2</em> from a Gaussian
     *  (normal) distribution with mean <em>0</em> and standard deviation
     *  <em>1</em> is defined as 
     *  \f[
     *     \begin{array}{rcl}
     *       y_1 & = & \sqrt{-2 \ln x_1} \cos(2\pi x_2) \		\
     *       y_2 & = & \sqrt{-2 \ln x_1} \sin(2\pi x_2).\		\
     *     \end{array}
     *  \f]
     *  The computation can be simplified by using the \em polar form of
     *  the Box-Muller transform. Instead of picking <em>x1</em> and
     *  <em>x2</em> uniformly inside the unit square, <em>v1</em> and
     *  <em>v2</em> are picked as the ordinate and abscissa of a random
     *  point inside the unit circle around the origin. Then, <em>R = v1 *
     *  v1 + v2 * v2</em> is a uniform sample and can be used instead of
     *  <em>x1</em>. Furthermore, the random angle <em>2PI * x2</em> can
     *  be replaced by the angle that the point <em>(v1, v2)</em> defines
     *  w.r.t. the <em>v1</em> axis. In this way, the expensive calls to
     *  <em>cos</em> and <em>sin</em> can be written as <em>v1 /
     *  sqrt(R)</em> and <em>v2 / sqrt(R)</em>.
     *  \n\n  
     *  A sample <em>y</em> from  a Gaussian (normal) distribution with
     *  mean <em>0</em> and standard deviation <em>1</em> is transformed
     *  into a sample <em>z</em> from a Gaussian (normal) distribution
     *  with mean <em>mean</em> and standard deviation <em>stddev</em> as 
     *  follows: 
     *  <CENTER><em>z = y * stddev + mean</em></CENTER>
     *
     *@par References:
     *    - Chapter 7 of Numerical Recipes in C.
     *    - G. E. P Box and M. E. Muller. <em>''A note on the generation of random normal deviates''</em>, 
     *      in <em>Annals Math. Stat</em>, V. 29, pp. 610-611, 1958.
     */
    double RandomGaussianReal(const double mean, const double stddev);

    /**
     *END Generate real values from gaussian distributions
     *@}
     */

    void RandomPointOnSphereSurface(const int nrDims,
                                    const double center[],
                                    const double r,
                                    double p[]);

    void RandomPointInsideSphere(const int nrDims,
                                 const double center[],
                                 const double r,
                                 double p[]);

    void RandomPointOnEllipsoidSurface(const int nrDims,
                                       const double center[],
                                       const double radii[],
                                       double p[]);

    void RandomPointInsideEllipsoid(const int nrDims,
                                    const double center[],
                                    const double radii[],
                                    double p[]);
}

#endif











