Download Link: https://assignmentchef.com/product/solved-csci5980-assignment5-bundle-adjustment
<br>
<h1>1             Data Capture and Overview</h1>

Figure 1: You will capture your cellphone images to reconstruct camera pose and 3D points.

In this assignment, you will use your cellphone images (more than 5) to reconstruct 3D camera poses and points with full bundle adjustment. Make sure you have enough baseline (translation) between images for well conditioned fundamental matrix while retaining enough number of correspondences between image. Avoid a scene dominated by a planar surface, i.e., the images need to contain many 3D objects as shown in Figure 1.

You will write a full pipeline of the structure from motion algorithm including matching, camera pose estimation using fundamental matrix, PnP, triangulation, and bundle adjustment. A nonlinear optimization is always followed by the initial estimate by linear least squares solution. The pipeline is described in Algorithm 1.

<strong>Algorithm 1 </strong>Structure from Motion

1: [Mx, My] = GetMatches(I<sub>1</sub>, ···, I<em><sub>N</sub></em>)

2: Normalize coordinate in Mx and My, i.e., <strong>x </strong>= <strong>K</strong><sup>−1</sup><strong>x</strong>.

3: Select two images I<em><sub>i</sub></em><sub>1 </sub>and I<em><sub>i</sub></em><sub>2 </sub>for the initial pair reconstruction.

4: [<strong>R</strong>, <strong>C</strong>, <strong>X</strong>] = CameraPoseEstimation([Mx(:,<em>i</em>1) My(:,<em>i</em>1)], [Mx(:,<em>i</em>2) My(:,<em>i</em>2)])

5: P = {<strong>P</strong><sub>1</sub>,<strong>P</strong><sub>2</sub>} where <strong>P</strong>

6: R = {<em>i</em>1<em>,i</em>2}

7: <strong>while </strong>|R| <em>&lt; N </em><strong>do</strong>

8:                 <em>i </em>= GetBestFrame(Mx, My, R);

9:                  [<strong>R</strong><em><sub>i</sub></em>, <strong>C</strong><em><sub>i</sub></em>] = PnP RANSAC([Mx(:,<em>i</em>) My(:,<em>i</em>)], <strong>X</strong>)

10:                    [<strong>R</strong><em><sub>i</sub></em>, <strong>C</strong><em><sub>i</sub></em>] = PnP Nonlinear(<strong>R</strong><em><sub>i </sub></em><strong>C</strong><em><sub>i</sub></em>, [Mx(:,<em>i</em>) My(:,<em>i</em>)], <strong>X</strong>)

11:            <strong>P</strong>

12:             <strong>for </strong>f = 1 : |R| <strong>do</strong>

13:                             U = FindUnreconstructedPoints(<strong>X</strong>, R<em><sub>f</sub></em>, <em>i</em>, Mx, My)

14:                     <strong>for </strong>j = 1 : |U| <strong>do</strong>

15:                                    <strong>u </strong>= [Mx(U<em><sub>j</sub></em>, <em>i</em>), My(U<em><sub>j</sub></em>, <em>i</em>)] and <strong>v </strong>= [Mx(U<em><sub>j</sub></em>, R<em><sub>f</sub></em>), My(U<em><sub>j</sub></em>, R<em><sub>f</sub></em>)]

16:                                    <strong>x </strong>= LinearTriangulation(<strong>u</strong>, <strong>P</strong><em><sub>i</sub></em>, <strong>v</strong>, <strong>P</strong><sub>R</sub><em><sub>f</sub></em>)

17:                                          <strong>x </strong>= NonlinearTriangulation(<strong>X</strong>, <strong>u</strong>, <strong>R</strong><em><sub>i</sub></em>, <strong>C</strong><em><sub>i</sub></em>, <strong>v</strong>, <strong>R</strong><sub>R</sub><em><sub>f</sub></em>, <strong>C</strong><sub>R</sub><em><sub>f</sub></em>)

18:                            <strong>X </strong>= <strong>X </strong>∪ <strong>x</strong>

19:                   <strong>end for</strong>

20:           <strong>end for</strong>

21:                P = P ∪ <strong>P</strong><em><sub>i </sub></em>and R = R ∪ <em>i</em>.

22:                   [P, <strong>X</strong>] = BundleAdjustment(P, <strong>X</strong>, R, Mx, My)

23: <strong>end while</strong>

<h1>2             Matching</h1>

Given a set of images, I<sub>1</sub><em>,</em>··· <em>,</em>I<em><sub>N</sub></em>, you will find matches across all images where <em>N </em>is the number of images similar to HW #4. Pick a reference image, I<em><sub>ref</sub></em>, and match with other images using SIFT features from VLFeat, i.e., I<em><sub>ref </sub></em>&#x2194; I<sub>1</sub><em>,</em>··· <em>,</em>I<em><sub>ref </sub></em>&#x2194; I<em><sub>N </sub></em>(no need to match I<em><sub>ref </sub></em>&#x2194; I<em><sub>ref</sub></em>).

Your matches are outlier free, i.e., bidirectional knn match → ratio test → inliers from the fundamental matrix based RANSAC. Based on the matches, you will build a measurement matrix, Mx and My:

[Mx, My] = GetMatches(I<sub>1</sub>, ···, I<em><sub>N</sub></em>)

Mx: F×N matrix storing x coordinate of correspondences

My: F×N matrix storing y coordinate of correspondences

The <em>f</em><sup>th </sup>feature point in image I<em><sub>i </sub></em>corresponds to a point in image I<em><sub>j</sub></em>. The x and y coordinates of the correspondence is stored at (<em>f,i</em>) and (<em>f,j</em>) elements in Mx and My, respectively. If (<em>f,i</em>) does not correspond to any point in image I<em><sub>k</sub></em>, you set -1 to indicate no match as shown in Figure 2.

Important: For better numerical stability, you can transform the measurements to the normalized coordinate by multiplying <strong>K</strong><sup>−1</sup>, i.e., <strong>x </strong>= <strong>K</strong><sup>−1</sup><strong>x </strong>where <strong>x </strong>is 2D measured points in homogeneous coordinate. You can run structure from motion in the normalized coordinate by factoring out <strong>K</strong>. When visualizing projection in the image, the coordinate needs to be transformed back to original coordinate by multiplying <strong>K</strong>.

<table width="0">

 <tbody>

  <tr>

   <td rowspan="2" width="17"><em>f</em></td>

   <td width="28"><em>x</em></td>

   <td rowspan="2" width="49"><em>x</em><em>f </em>,<em>i               f</em>Mx</td>

   <td rowspan="2" width="25">, <em><sub>j </sub></em>−</td>

   <td rowspan="2" width="10">1</td>

   <td rowspan="2" width="35"><em>f</em></td>

   <td width="28"><em>y</em></td>

   <td rowspan="2" width="48"><em>y</em><em>f </em>,<em>i</em>My</td>

   <td rowspan="2" width="21"><em>f </em>, <em>j </em>−</td>

   <td rowspan="2" width="13">1</td>

  </tr>

  <tr>

   <td width="28"> </td>

   <td width="28"> </td>

  </tr>

 </tbody>

</table>

<em>N                                        N</em>

<em>F</em>

(<em>x</em><em>f </em>,<em>i </em>, <em>y </em><em>f </em>,<em>i </em>)&#x2194;(<em>x</em><em>f </em>, <em>j </em>, <em>y </em><em>f </em>, <em>j </em>)&#x2194;(<em>x</em><em>f </em>,<em>k </em>, <em>y </em><em>f </em>,<em>k </em>)

Figure 2: The <em>f</em><sup>th </sup>feature point in image I<em><sub>i </sub></em>corresponds to a point in image I<em><sub>j</sub></em>. The x and y coordinates of the correspondence is stored at (<em>f,i</em>) and (<em>f,j</em>) elements in Mx and My, respectively. If (<em>f,i</em>) does not correspond to any point in image I<em><sub>k</sub></em>, you set -1 to indicate no match.

<h1>3             Camera Pose Estimation</h1>

You will write a camera pose estimation code that takes correspondences between two images, I<em><sub>i</sub></em><sub>1 </sub>and I<em><sub>i</sub></em><sub>2 </sub>where <em>i</em>1 and <em>i</em>2 are the indices of the initial images to reconstruct selected manually.

[<strong>R</strong>, <strong>C</strong>, <strong>X</strong>] = CameraPoseEstimation(<strong>u</strong><sub>1</sub>, <strong>u</strong><sub>2</sub>)

<strong>R </strong>and <strong>C</strong>: the relative transformation of the <em>i</em>2 image <strong>u</strong><sub>1 </sub>and <strong>u</strong><sub>2</sub>: 2D-2D correspondences

As studied in HW #4, you will compute:

<ol>

 <li>Fundamental matrix via RANSAC on correspondences, Mx(:,<em>i</em>1), My(:,<em>i</em>2)</li>

 <li>Essential matrix from the fundamental matrix</li>

 <li>Four configurations of camera poses given the essential matrix</li>

 <li>Disambiguation via chierality (using 3D point linear triangulation): <strong>X </strong>= LinearTriangulation(<strong>u</strong>, <strong>P</strong><em><sub>i</sub></em>, <strong>v</strong>, <strong>P</strong><em><sub>j</sub></em>)</li>

</ol>

<strong>Write-up:</strong>

Figure 3: Camera pose estimation.

<ul>

 <li>Visualize inlier matches as shown in Figure 3(a).</li>

 <li>Visualize camera pose and 3D reconstructed points in 3D as shown in Figure 3(b).</li>

</ul>

<h1>4             Nonlinear 3D Point Refinement</h1>

You will write a nonlinear triangulation code. Given the linear estimate for the point

T triangulation, <strong>X</strong>, you will refine the 3D point <strong>X</strong>to minimize geometric error (reprojection error) via iterative nonlinear least squares estimation,

T                  !−1                             T

<em>∂f</em>(<strong>X</strong>) <em>∂f</em>(<strong>X</strong>)             <em>∂f</em>(<strong>X</strong>)

∆<strong>X </strong>=           (<strong>b </strong>− <em>f</em>(<strong>X</strong>))<em>.       </em>(1) <em>∂</em><strong>X </strong><em>∂</em><strong>X       </strong><em>∂</em><strong>X</strong>

<strong>Write-up:</strong>

<ul>

 <li>Derive the point Jacobian, i.e., <em><sup>∂f</sup></em><em><sub>∂</sub></em><sup>(</sup><strong><sub>X</sub><sup>X</sup></strong><sup>)</sup><em><sup>j </sup></em>and write the following code. df dX = JacobianX(<strong>K</strong>, <strong>R</strong>, <strong>C</strong>, <strong>X</strong>)</li>

 <li>Write a code to refine the 3D point by minimizing the reprojection error and visualize reprojection error reduction similar to Figure 5.</li>

</ul>

<strong>X </strong>= NonlinearTriangulation(<strong>X</strong>, <strong>u</strong><sub>1</sub>, <strong>R</strong><sub>1</sub>, <strong>C</strong><sub>1</sub>, <strong>u</strong><sub>2</sub>, <strong>R</strong><sub>2</sub>, <strong>C</strong><sub>2</sub>)

<strong>Algorithm 2 </strong>Nonlinear Point Refinement

<table width="0">

 <tbody>

  <tr>

   <td width="360">T1:2: <strong>for </strong>j = 1 : nIters <strong>do</strong>3:          Build point Jacobian, 4:            Compute <em>f</em>(<strong>X</strong>).5:6:                 <strong>X </strong>= <strong>X </strong>+ ∆<strong>X</strong>7: <strong>end for</strong></td>

   <td width="229">.</td>

  </tr>

 </tbody>

</table>

<h1>5             Camera Registration</h1>

You will register an additional image, I<em><sub>j </sub></em>using 2D-3D correspondences.

<strong>Write-up:</strong>

<ul>

 <li>(3D-2D correspondences) Given 3D triangulated points, find 2D-3D matches, <strong>X </strong>&#x2194; <strong>u</strong>.</li>

 <li>(Perspective-n-Point algorithm) Write a code that computes 3D camera pose from 3D-2D correspondences:</li>

</ul>

[<strong>R</strong>, <strong>C</strong>] = LinearPnP(<strong>u</strong>, <strong>X</strong>)

<strong>X</strong>: <em>n </em>× 3 matrix containing <em>n </em>3D reconstructed points <strong>u</strong>: <em>n </em>× 2 matrix containing <em>n </em>2D points in the additional image I<sub>3 </sub><strong>R </strong>and <strong>C</strong>: rotation and translation for the additional image.

<em>Hint: </em>After the linear solve, rectify the rotation matrix such that det(<strong>R</strong>) = 1 and scale <strong>C </strong>according to the rectification.

<ul>

 <li>(RANSAC PnP) Write a RANSAC algorithm for the camera pose registration (PnP) given <em>n </em>matches using the following pseudo code:</li>

</ul>

<strong>Algorithm 3 </strong>PnP RANSAC

1: <em>nInliers </em>← 0

2: <strong>for </strong><em>i </em>= 1 : <em>M </em><strong>do</strong>

3:               Choose 6 correspondences, <strong>X</strong><em><sub>r </sub></em>and <strong>u</strong><em><sub>r</sub></em>, randomly from <strong>X </strong>and <strong>u</strong>.

4:                        [<strong>R</strong><em><sub>r</sub></em>, <strong>t</strong><em><sub>r</sub></em>] = LinearPnP(<strong>u</strong><em><sub>r</sub></em>, <strong>X</strong><em><sub>r</sub></em>)

5:                Compute the number of inliers, <em>n<sub>r</sub></em>, with respect to <strong>R</strong><em><sub>r</sub></em>, <strong>t</strong><em><sub>r</sub></em>.

6: <strong>if </strong><em>n<sub>r </sub>&gt; nInliers </em><strong>then </strong>7: <em>nInliers </em>← <em>n<sub>r</sub></em>

8:                         <strong>R </strong>= <strong>R</strong><em><sub>r </sub></em>and <strong>t </strong>= <strong>t</strong><em><sub>r</sub></em>

9:              <strong>end if</strong>

10: <strong>end for</strong>

Visualize 3D registered pose as shown in Figure 4.

(a) Front view                                                                    (b) Top view

Figure 4: Additional image registration.

(4) (Reprojection) Visualize measurement and reprojection to verify the solution.

<h1>6             Nonlinear Camera Refinement</h1>

Given the initial estimate <strong>R</strong><em><sub>i </sub></em>and <strong>t</strong><em><sub>i</sub></em>, you will refine the camera pose to minimize geometric error (reprojection error) via iterative nonlinear least squares estimation,

<table width="0">

 <tbody>

  <tr>

   <td width="363">                              T                 !<sup>−1                           </sup>T<em>∂f</em>(<strong>p</strong>) <em>∂f</em>(<strong>p</strong>)           <em>∂f</em>(<strong>p</strong>)∆<strong>p </strong>=         (<strong>b </strong>− <em>f</em>(<strong>p</strong>))<em>, ∂</em><strong>p        </strong><em>∂</em><strong>p       </strong><em>∂</em><strong>p</strong></td>

   <td width="126"> </td>

  </tr>

  <tr>

   <td width="363">   <em>u</em>1<em>/w</em>1</td>

   <td width="126">  <em>x</em><sub>1</sub></td>

  </tr>

 </tbody>

</table>

 <em>v</em>1<em>/w</em>1  <em>y</em><sub>1 </sub>

 … <em>,,                                       </em><strong>b </strong>=  …  (2)

<em>f</em>(<strong>p</strong>) =  

 <em>u</em><em>n/w</em><em>n </em> <em>x<sub>n </sub></em>



<em>v</em><em>n/w</em><em>n                                                                                                                                                                                  y<sub>n</sub></em>

<table width="0">

 <tbody>

  <tr>

   <td width="550">quaternion representation of the camera rotation.It is possible to minimize the overshooting by adding damping, <em>λ </em>as follows:</td>

   <td width="39"> </td>

  </tr>

  <tr>

   <td width="550">                                                                     T                             !−1                           T<em>∂f</em>(<strong>p</strong>) <em>∂f</em>(<strong>p</strong>)                    <em>∂f</em>(<strong>p</strong>)∆<strong>p </strong>=                               + <em>λ</em><strong>I                       </strong>(<strong>b </strong>− <em>f</em>(<strong>p</strong>))<em>,</em></td>

   <td width="39">(3)</td>

  </tr>

 </tbody>

</table>

where <strong>p </strong><strong> C</strong><sup>T </sup><strong>q</strong><sup>T T</sup>. <strong>C </strong>∈ R<sup>3 </sup>is the camera optical center and <strong>q </strong>∈ S<sup>3 </sup>is the

<em>∂</em><strong>p        </strong><em>∂</em><strong>p                       </strong><em>∂</em><strong>p</strong>

where <em>λ </em>is the damping parameter. You can try <em>λ </em>∈ [0<em>,</em>10].

Note that the conversion between quaternion and rotation matrix is given as follows:

<strong>R </strong><em> ,</em>

<strong>q </strong><em> ,        </em>where <strong>Write-up:</strong>

<ul>

 <li>Derive the quaternion Jacobian to rotation using Equation (4), i.e., <em><u><sup>∂</sup></u><sub>∂</sub></em><strong><u><sup>R</sup></u><sub>q </sub></strong>and write the following code. Note: ignore the normalization k<strong>q</strong>k = 1. dR dq = JacobianQ(<strong>q</strong>)</li>

 <li>Derive the rotation Jacobian to projection using Equation (2), i.e., <em><sup>∂f</sup></em><em><sub>∂</sub></em><sup>(</sup><strong><sub>R</sub><sup>p</sup></strong><sup>)</sup><em><sup>j </sup></em>where</li>

</ul>

T and write the following code. Note: use vectorized form of

the rotation matrix.

df dR = JacobianR(<strong>R</strong>, <strong>C</strong>, <strong>X</strong>)

<ul>

 <li>Derive the expression of <em><sup>∂f</sup><sub>∂</sub></em><sup>(</sup><strong><sub>q</sub><sup>p</sup></strong><sup>)</sup><em><sup>j </sup></em>using the chain rule.</li>

 <li>Derive the camera center Jacobian to projection using Equation (2), i.e., <em><sup>∂f</sup></em><em><sub>∂</sub></em><sup>(</sup><strong><sub>C</sub><sup>p</sup></strong><sup>)</sup><em><sup>j </sup></em>and write the following code.</li>

</ul>

df dC = JacobianC(<strong>R</strong>, <strong>C</strong>, <strong>X</strong>)

<ul>

 <li>Write a code to refine the camera pose by minimizing the reprojection error and visualize reprojection error reduction as shown in Figure 5:</li>

</ul>

[<strong>R</strong>, <strong>C</strong>] = PnP Nonlinear(<strong>R C</strong>, <strong>u</strong>, <strong>X</strong>)

<table width="0">

 <tbody>

  <tr>

   <td width="663"><strong>Algorithm 4 </strong>Nonlinear Camera Pose Refinement</td>

   <td width="3"> </td>

  </tr>

  <tr>

   <td width="663">1: <strong> C</strong>T <strong>q</strong>T T2: <strong>for </strong>j = 1 : nIters <strong>do</strong>3:                    <strong>C </strong>= <strong>p</strong><sub>1:3</sub>, <strong>R</strong>=Quaternion2Rotation(<strong>q</strong>), <strong>q </strong>= <strong>p</strong><sub>4:7</sub>4:          Build camera pose Jacobian for all points, 5:            Compute <em>f</em>(<strong>p</strong>).6:                                                                                                                )) using Equation (3).7:               <strong>p </strong>= <strong>p </strong>+ ∆<strong>p</strong>8:               Normalize the quaternion scale, <strong>p</strong><sub>4:7 </sub>= <strong>p</strong><sub>4:7</sub><em>/</em>k<strong>p</strong><sub>4:7</sub>k.9: <strong>end for</strong></td>

   <td width="3">.</td>

  </tr>

 </tbody>

</table>

Figure 5: Nonlinear refinement reduces the reprojection error (0.19→0.11).

<h1>7             Bundle Adjustment</h1>

You will write a nonlinear refinement code that simultaneously optimizes camera poses and 3D points using the sparse nature of the Jacobian matrix. [P, <strong>X</strong>] = BundleAdjustient(P, <strong>X</strong>, R, Mx, My)

For example, consider 3 camera poses and 2 points. The Jacobian matrix can be written as follows:

<strong>0</strong>2×7

2               <em><sub>∂f                  </sub></em>1

<strong>JJ<sub>p </sub>J<sub>X </sub></strong><sup> </sup>(5)

<sup></sup> <strong>0</strong>2×7                                                         <em><sub>∂</sub></em><strong>0</strong>2×3

<strong>                           0</strong>2×3

where <strong>J<sub>p </sub></strong>and <strong>J<sub>X </sub></strong>are the Jacobian for camera and point, respectively, and <em>λ </em>∈ [0<em>,</em>10].

The normal equation, <strong>J</strong><sup>T</sup><strong>J</strong>∆<strong>x </strong>= <strong>J</strong><sup>T</sup>(<strong>b </strong>− <em>f</em>(<strong>x</strong>)) can be decomposed into:

<sup> </sup><strong>A                       Be<sup>p </sup></strong>

=               <em>,                                          </em>(6)

<strong>B</strong><sup>T </sup><strong>D            </strong>∆<strong>X              e<sub>X</sub></strong>

where

<strong>A </strong>= <strong>J</strong><sup>T</sup><strong><sub>p</sub>J<sub>p </sub></strong>+ <em>λ</em><strong>I</strong><em>,         </em><strong>B </strong>= <strong>J</strong><sup>T</sup><strong><sub>p</sub>J<sub>X</sub></strong><em>,          </em><strong>D </strong>= <strong>J</strong><sup>T</sup><strong><sub>X</sub>J<sub>X </sub></strong>+ <em>λ</em><strong>I e<sub>p </sub></strong>= <strong>J</strong><sup>T</sup><strong><sub>p</sub></strong>(<strong>b </strong>− <em>f</em>(<strong>x</strong>))<em>,          </em><strong>e<sub>X </sub></strong>= <strong>J<sub>X</sub></strong><sup>T</sup>(<strong>b </strong>− <em>f</em>(<strong>x</strong>))

where  and <strong> X</strong> where <em>I </em>and <em>M </em>are the number of images and points, respectively.

The decomposed normal equation in Equation (6) allows us to efficiently compute the inverse of <strong>J</strong><sup>T</sup><strong>J </strong>using Schur complement of <strong>D</strong>:

∆<strong>p</strong>b = (<strong>A </strong>− <strong>BD</strong><sup>−1</sup><strong>B</strong><sup>T</sup>)<sup>−1</sup>(<strong>e<sub>p </sub></strong>− <strong>BD</strong><sup>−1</sup><strong>e<sub>X</sub></strong>)<em>,</em>

where <strong>D </strong>is a block diagonal matrix whose inverse can be efficiently computed by inverting small block matrix:

<table width="0">

 <tbody>

  <tr>

   <td width="82"><strong>d</strong><sub>1</sub><strong>D </strong>= </td>

   <td width="31">…</td>

   <td width="81"><em>,</em> <strong>d</strong><em><sub>M</sub></em></td>

   <td width="106"> −1 <strong>d</strong><sub>1</sub><strong>D</strong>−1 =  </td>

   <td width="31">…</td>

   <td width="130">−1<strong>d</strong><em><sub>M</sub></em></td>

   <td width="21">(7)</td>

  </tr>

 </tbody>

</table>

The bundle adjustment algorithm is summarized in Algorithm 5. Note that not all points are visible from cameras. You need to reason about the visibility, i.e., if the point is not visible from the camera, the corresponding Jacobian and measurement from <strong>J </strong>and <strong>b </strong>will be omitted, respectively.

<strong>Algorithm 5 </strong>Bundle Adjustment

<table width="0">

 <tbody>

  <tr>

   <td colspan="7" width="732">T1:and2: <strong>for </strong>iter = 1 : nIters <strong>do</strong>3:                    Empty <strong>J<sub>p</sub></strong>, <strong>J<sub>X</sub></strong>, <strong>b</strong>, <strong>f</strong>, <strong>D</strong><sub>inv</sub>.</td>

  </tr>

  <tr>

   <td width="40">4:</td>

   <td colspan="6" width="692"><strong>for </strong><em>i </em>= 1 : <em>M </em><strong>do</strong></td>

  </tr>

  <tr>

   <td width="40">5:</td>

   <td colspan="6" width="692"><strong>d </strong>= <strong>0</strong>3×3</td>

  </tr>

  <tr>

   <td width="40">6:</td>

   <td colspan="6" width="692"><strong>for </strong><em>j </em>= 1 : <em>I </em><strong>do</strong></td>

  </tr>

  <tr>

   <td width="40">7:</td>

   <td colspan="6" width="692"><strong>if </strong>the <em>i</em><sup>th </sup>point is visible from the <em>j</em><sup>th </sup>image <strong>then</strong></td>

  </tr>

  <tr>

   <td width="40">8:</td>

   <td colspan="4" width="404"><strong>J</strong>1 = <strong>0</strong>2×7<em>I </em>and <strong>J</strong>2 = <strong>0</strong>2×3<em>M</em></td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">9:</td>

   <td width="120"><strong>J</strong></td>

   <td width="20"> </td>

   <td width="121"> </td>

   <td width="142"></td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">10:</td>

   <td width="120"><strong>J</strong></td>

   <td width="20"> </td>

   <td width="121"> </td>

   <td width="142"></td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">11:</td>

   <td width="120"><strong>J<sub>p </sub></strong></td>

   <td width="20"><strong>J</strong>T<strong><sub>p</sub></strong></td>

   <td width="121"><strong>J</strong></td>

   <td width="142">Tand <strong>J<sub>X</sub></strong></td>

   <td width="29"><strong>J</strong>T<strong><sub>X</sub></strong></td>

   <td width="260">T <strong>J</strong></td>

  </tr>

  <tr>

   <td width="40">12:</td>

   <td width="120"><strong>d</strong></td>

   <td width="20"> </td>

   <td width="121"></td>

   <td width="142"> </td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">13:</td>

   <td colspan="2" width="140"><strong>b </strong>=  <strong>b</strong>T</td>

   <td width="121"><strong>u</strong></td>

   <td width="142"> </td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">14:</td>

   <td colspan="2" width="140"><strong>f </strong><strong> f</strong>T</td>

   <td width="121"><strong>x</strong></td>

   <td width="142">where</td>

   <td width="29"> </td>

   <td width="260"><strong>I </strong></td>

  </tr>

  <tr>

   <td width="40">15:</td>

   <td colspan="2" width="140"><strong>end if</strong></td>

   <td width="121"> </td>

   <td width="142"> </td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">16:</td>

   <td colspan="2" width="140"><strong>end for</strong></td>

   <td width="121"> </td>

   <td width="142"> </td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">17:</td>

   <td colspan="2" width="140"><strong>d </strong>= <strong>d </strong>+ <em>λ</em><strong>I</strong></td>

   <td width="121"> </td>

   <td width="142"> </td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">18:</td>

   <td colspan="4" width="404"><strong>D</strong>inv = blkdiag(<strong>D</strong>inv, <strong>d</strong>−1)</td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">19:</td>

   <td colspan="4" width="404"><strong>end for</strong></td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">20:</td>

   <td colspan="4" width="404"><strong>e<sub>p </sub></strong>= <strong>J</strong><sup>T</sup><strong><sub>p</sub></strong>(<strong>b </strong>− <strong>f</strong>)</td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">21:</td>

   <td colspan="4" width="404"><strong>e<sub>X </sub></strong>= <strong>J</strong><sup>T</sup><strong><sub>X</sub></strong>(<strong>b </strong>− <strong>f</strong>)</td>

   <td width="29"> </td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">22:</td>

   <td colspan="5" width="432"><strong>A </strong>= <strong>J</strong>T<strong>pJ</strong><strong>p </strong>+ <em>λ</em><strong>I</strong>, <strong>B </strong><strong>J</strong><strong>X</strong>, <strong>D</strong>−1 = <strong>D</strong>inv</td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">23:</td>

   <td colspan="5" width="432">∆<strong>p </strong>= (<strong>A </strong>− <strong>BD</strong>−1<strong>B</strong>T)−1(<strong>e</strong><strong>p </strong>− <strong>BD</strong>−1<strong>e</strong><strong>X</strong>) b</td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td width="40">24:</td>

   <td colspan="5" width="432">Normalize quaternions.</td>

   <td width="260"> </td>

  </tr>

  <tr>

   <td colspan="7" width="732">25:<strong>B</strong><sup>T</sup>∆<strong>p</strong>) b26: <strong>end for</strong></td>

  </tr>

 </tbody>

</table>

<strong>Write-up: </strong>You will first start with two images and 10 3D points to test your bundle adjustment program.

<ul>

 <li>Derive <strong>J<sub>p </sub></strong>and <strong>J<sub>X</sub></strong>.</li>

 <li>Run Algorithm 5 and visualize the reprojection error similar to Figure 5.</li>

</ul>

<h1>8             Putting All Things Together</h1>

<strong>Write-up: </strong>You will run with all images and 3D points based on Algorithm 1.

<ul>

 <li>Visualize 3D camera pose and points as shown in Figure 6.</li>

 <li>Visualize reprojection for all images.</li>

</ul>

Figure 6: You will reconstruct all images and 3D points using structure from notion.