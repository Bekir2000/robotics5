#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


#include <rl/plan/Sampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        /**
         * Uniform random sampling strategy.
         */
        class YourSampler : public Sampler
        {
        public:
            YourSampler();

            virtual ~YourSampler();

            ::rl::math::Vector generate();

            virtual void seed(const ::std::mt19937::result_type& value);

			bool useNormalDistribution = true;

        protected:
            ::std::uniform_real_distribution< ::rl::math::Real>::result_type rand();

            ::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
			::std::normal_distribution<::rl::math::Real> normalDistribution;

            ::std::mt19937 randEngine;
			::std::mt19937 normalEngine;

        private:

        };
    }
}


#endif // _YOURSAMPLER_H_
