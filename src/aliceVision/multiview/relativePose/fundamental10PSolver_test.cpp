// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/multiview/relativePose/Fundamental10PSolver.hpp>

#define BOOST_TEST_MODULE essentialF10Solver
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include <vector>

using namespace aliceVision;
using namespace aliceVision::multiview;

BOOST_AUTO_TEST_CASE(Fundamental10PSolver_8_solutions)
{
  // input data
  Mat X = Mat(10, 2);
  X <<  1.425203022203948e-01,  1.802107961554276e-02,
       -2.467473080283717e-01,  1.134013928865132e-01,
       -2.165429045024671e-01,  1.016561086554276e-01,
       -1.779508570620888e-01, -5.443423622532895e-02,
       -9.503401906866776e-02, -1.912260035464638e-01,
        4.285402960526316e-01,  8.614113255550987e-02,
       -3.182964124177631e-02, -1.139683291786595e-01,
       -8.882934570312501e-02,  9.867373817845394e-02,
        3.867745811060855e-01,  1.629066868832237e-02,
        4.764409436677632e-01, -8.015930175781250e-02;

  Mat U = Mat(10, 2);
  U <<  1.470885587993421e-01, -3.716578433388158e-02,
       -3.384269955283717e-01,  3.505518863075658e-02,
       -2.903711258737665e-01,  2.336130242598684e-02,
       -2.017722360711349e-01, -1.733409359580592e-01,
       -1.022310598273026e-01, -1.984814292506168e-01,
        4.136125745271382e-01,  5.560238486842106e-02,
       -4.780700683593750e-02, -1.279896786338405e-01,
       -1.000526829769737e-01,  3.618122301603619e-02,
        3.628858064350329e-01, -6.060566149259868e-03,
        4.513226639597039e-01, -1.025031481291119e-01;

  // transpose for aliceVision
  X.transposeInPlace();
  U.transposeInPlace();

  // expected result
  std::vector<Mat3> resF;
  Mat3 resF1; resF1 << -5.751087019565978e+02, -4.158188692023641e+02, -3.933617847277337e+01,  5.623077923604983e+02, -1.155037417506222e+03, -3.603293875557833e+01, -5.277848487166921e-01, -9.666992599768776e-02, 1.000000000000000e+00; resF.push_back(resF1);
  Mat3 resF2; resF2 << -2.199589063941379e+04, -2.847966717222817e+04, -1.568596434350923e+02,  2.668165615957419e+04,  3.794695160371831e+04,  3.582097175710577e+02, -1.985243717996453e+03,  3.206256740219327e+03, 1.000000000000000e+00; resF.push_back(resF2);
  Mat3 resF3; resF3 << -2.406164718504593e+02,  1.051858077445098e+03,  1.029257202497226e+02, -9.384606808289793e+02, -1.561769129182295e+00, -7.588146214289489e+00, -2.671999077624011e+01,  1.072299253095423e+00, 1.000000000000000e+00;	resF.push_back(resF3);
  Mat3 resF4; resF4 <<  4.773291645793250e+02, -7.340046095681545e+03, -7.622433073293142e+01,  7.353659444399805e+03, -7.228144265103590e+02,  2.313924014459177e+01, -4.096068639840655e+02,  1.760210427160575e+02, 1.000000000000000e+00; resF.push_back(resF4);
  Mat3 resF5; resF5 << -3.634107580978586e+00, -6.861915294293681e+01, -1.419442789733012e+00,  1.043103524635315e+02, -9.905128840695012e+01, -1.288978514515288e+01, -5.004364829690772e+00,  1.759690666207339e+01, 1.000000000000000e+00; resF.push_back(resF5);
  Mat3 resF6; resF6 <<  3.845891112065496e+02, -3.376624483974341e+02,  1.058628706262809e+00,  4.567090509397883e+02, -1.782032107508239e+01,  6.603245933354176e-01, -3.057044617411873e+01, -1.171693147510399e+01, 1.000000000000000e+00; resF.push_back(resF6);
  Mat3 resF7; resF7 <<  9.173705728965071e-01,  6.566802897577452e+02, -9.150842187178472e+01, -6.985651797488533e+02,  6.389672979364663e+01, -2.813972637838619e+01,  1.224499375450345e+02,  2.273467046355762e+00, 1.000000000000000e+00; resF.push_back(resF7);
  Mat3 resF8; resF8 <<  9.260243587671360e+00,  2.901973712182848e+01, -4.525967397412600e+00,  3.873064498047484e+00, -3.308114531934898e+01, -1.268705935682481e+01,  3.742963523739931e+00,  1.576531480674961e+01, 1.000000000000000e+00; resF.push_back(resF8);

  std::vector<Mat2X> resL;
  Mat2X resL1(2, 1); resL1 <<  6.664890785418972e+02,  9.360374635703592e-01; resL.push_back(resL1);
  Mat2X resL2(2, 1); resL2 << -1.401232955798790e+02,  1.190659194737138e+03; resL.push_back(resL2);
  Mat2X resL3(2, 1); resL3 <<  5.478399615715365e+01,  1.277075963855622e+01; resL.push_back(resL3);
  Mat2X resL4(2, 1); resL4 << -3.004792949356860e+01,  1.442424098845748e+02; resL.push_back(resL4);
  Mat2X resL5(2, 1); resL5 <<  5.619131137084262e+00,  1.528997232119745e+01; resL.push_back(resL5);
  Mat2X resL6(2, 1); resL6 << -1.359819306123174e+00, -3.194231629423156e+02; resL.push_back(resL6);
  Mat2X resL7(2, 1); resL7 << -7.668121663596985e+00, -1.015843563423398e+01; resL.push_back(resL7);
  Mat2X resL8(2, 1); resL8 << -8.478427431999348e+00, -4.469547181112368e+00; resL.push_back(resL8);

  // process
  std::vector<relativePose::Fundamental10PModel> models;
  relativePose::Fundamental10PSolver().solve(X, U, models);

  // test results
  if(resF.size() != models.size())
    BOOST_CHECK(false);

  for(Eigen::Index i = 0; i < resF.size(); ++i)
  {
    relativePose::Fundamental10PModel model = models.at(i);
    BOOST_CHECK(resF.at(i).isApprox(model.getMatrix(), 1e-1));
    BOOST_CHECK(resL.at(i).isApprox(model.getRadialDistortion(), 1e-1));
  }
}

BOOST_AUTO_TEST_CASE(Fundamental10PSolver_2_solutions)
{
  // input data
  Mat X = Mat(10, 2);
  X << -3.229677381013569e-01,  9.796846088610198e-02,
       -1.859613679584704e-01, -1.331652189555921e-02,
        3.993208393297698e-01, -4.309213738692434e-02,
        4.570184647409539e-01,  6.516200015419409e-02,
       -2.922935084292763e-02, -3.059326252184416e-01,
       -1.386923378392270e-01, -4.580717387952302e-02,
       -7.286097476356908e-02,  6.164560418379934e-02,
        5.396587171052632e-02,  9.743896484375000e-02,
        4.628304893092105e-01, -1.323552021227385e-01,
        4.939758300781250e-01, -4.290067973889802e-02;

  Mat U = Mat(10, 2);
  U << -4.438024018940173e-01,  3.003572162828947e-05,
       -2.212387888055099e-01, -1.264103939658717e-01,
        3.731781327097040e-01, -6.214079204358552e-02,
        4.439694053248355e-01,  3.183979235197369e-02,
        2.125557026110198e-01, -1.950167364823191e-01,
       -1.560874216180099e-01, -7.848533228824013e-02,
       -7.585523103412829e-02, -1.291279039884868e-02,
       -1.167459909539474e-01,  2.065792686060855e-02,
        4.302636076274671e-01, -1.466708052785773e-01,
        4.942827405427632e-01, -8.317700837787830e-02;

  // transpose for aliceVision
  X.transposeInPlace();
  U.transposeInPlace();

  // expected result
  std::vector<Mat3> resF;
  {
    Mat3 resF1;
    resF1 <<  1.732473500041804e+02,  7.711017146713161e+00,  9.471075243833084e+00, -1.518330376101678e+01, 3.516937871974938e+02, 1.097973146625093e+01, -1.046922253151639e-01,  6.610316585565101e-03, 1.000000000000000e+00;
    resF.push_back(resF1);

    Mat3 resF2;
    resF2 << -1.716387900591730e+04, -3.791505579727971e+04, -4.981406222447205e+00, 6.621894661079650e+03, 4.767612634161247e+04, -1.134635374165869e+02,  1.131438262492466e+03, -1.429200024334792e+04, 1.000000000000000e+00;
    resF.push_back(resF2);
  }

  std::vector<Mat2X> resL;
  {
    relativePose::Fundamental10PModel::Mat21 resL1;
    relativePose::Fundamental10PModel::Mat21 resL2;

    resL1 << -1.904715904949555e+02,  4.147643173367164e-01;
    resL.push_back(resL1);

    resL2 <<  1.258548483732838e+01, -2.918230091342369e+03;
    resL.push_back(resL2);
  }

  // process
  std::vector<relativePose::Fundamental10PModel> models;
  relativePose::Fundamental10PSolver().solve(X, U, models);

  // test results
  if(resF.size() != models.size())
    BOOST_CHECK(false);

  for(Eigen::Index i = 0; i < resF.size(); ++i)
  {
    relativePose::Fundamental10PModel model = models.at(i);
    BOOST_CHECK(resF.at(i).isApprox(model.getMatrix(), 1e-1));
    BOOST_CHECK(resL.at(i).isApprox(model.getRadialDistortion(), 1e-1));
  }
}
