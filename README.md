GOCO2025s Gravity Plugin for Gazebo
1. 개요

본 프로젝트는 Gazebo 시뮬레이터에서 기본적으로 제공되는 단순 중력 모델을 대체하여, 실제 위성 환경에 보다 근접한 고정밀 지구 중력장 모델을 적용하기 위한 플러그인이다.

Gazebo의 내부 물리 엔진(ODE, Bullet, DART)을 수정하지 않고, 외부에서 계산한 중력 가속도를 힘(force) 형태로 주입하는 방식으로 동작한다.

본 구현은 NASA/JPL에서 제공하는 GOCO2025s 구면조화(Spherical Harmonics) 중력 모델을 기반으로 한다.

2. 배경 및 필요성

Gazebo 기본 중력 모델은 균일한 중력장을 가정

실제 위성은 지구의 편평도(J2 효과) 및 질량 분포에 따른 중력 섭동을 경험

정밀 궤도 전파, 섭동 분석, 임무 검증을 위해 고차 중력 모델 필요

3. 시스템 구조

Plugin Type: Gazebo ModelPlugin

적용 대상: 위성 모델의 각 Link

중력 계산 방식:

GOCO2025s 구면조화 중력 퍼텐셜 계산

퍼텐셜 기울기 → 가속도 계산

가속도 → 힘(force) 변환 후 Gazebo에 적용

4. GOCO2025s 중력 모델 데이터
4.1 데이터 준비

NASA/JPL에서 제공하는 GOCO2025s 중력 모델 데이터 다운로드

다음 항목 포함 여부 확인:

구면조화 계수: C_{lm}, S_{lm}

사용할 최대 차수/계수 선택

권장 값: degree/order = 180 또는 360

계산 효율을 위해 불필요한 고차 항 제외

5. 중력 퍼텐셜 계산
5.1 구면조화 기반 퍼텐셜

중력 퍼텐셜은 구면조화 전개를 사용하여 계산된다.

5.2 완전 정규화 연관 르장드르 함수

퍼텐셜 계산에는 Fully-Normalized Associated Legendre Functions가 사용된다.

𝑃
ˉ
𝑛
,
𝑚
(
𝑥
)
=
𝑁
(
𝑛
,
𝑚
)
⋅
𝑃
𝑛
,
𝑚
(
𝑥
)
P
ˉ
n,m
	​

(x)=N(n,m)⋅P
n,m
	​

(x)


정규화 계수:

𝑁
(
𝑛
,
𝑚
)
=
(
2
−
𝛿
𝑚
0
)
⋅
(
2
𝑛
+
1
)
(
𝑛
−
𝑚
)
!
(
𝑛
+
𝑚
)
!
N(n,m)=(2−δ
m0
	​

)⋅
(n+m)!
(2n+1)(n−m)!
	​

	​




이 정규화 방식은 현대 중력장 모델의 표준과 일치하며 수치적 안정성을 보장한다.

6. 중력 가속도 계산

중력 가속도는 퍼텐셜의 기울기로부터 계산된다.

𝑎
=
−
∇
𝑉
a=−∇V


6.1 구면좌표계 가속도 성분

반경 방향: 
𝑎
𝑟
=
−
∂
𝑉
∂
𝑟
a
r
	​

=−
∂r
∂V
	​




위도 방향: 
𝑎
𝜙
=
−
1
𝑟
∂
𝑉
∂
𝜙
a
ϕ
	​

=−
r
1
	​

∂ϕ
∂V
	​




경도 방향: 
𝑎
𝜆
=
−
1
𝑟
cos
⁡
𝜙
∂
𝑉
∂
𝜆
a
λ
	​

=−
rcosϕ
1
	​

∂λ
∂V
	​




7. 좌표계 변환 (Spherical → Cartesian)

Gazebo는 직교 좌표계 
(
𝑥
,
𝑦
,
𝑧
)
(x,y,z)를 사용하므로 가속도 성분 변환이 필요하다.

𝑎
𝑥
	
=
𝑎
𝑟
cos
⁡
𝜙
cos
⁡
𝜆
−
𝑎
𝜙
sin
⁡
𝜙
cos
⁡
𝜆
−
𝑎
𝜆
sin
⁡
𝜆


𝑎
𝑦
	
=
𝑎
𝑟
cos
⁡
𝜙
sin
⁡
𝜆
−
𝑎
𝜙
sin
⁡
𝜙
sin
⁡
𝜆
+
𝑎
𝜆
cos
⁡
𝜆


𝑎
𝑧
	
=
𝑎
𝑟
sin
⁡
𝜙
+
𝑎
𝜙
cos
⁡
𝜙
a
x
	​

a
y
	​

a
z
	​

	​

=a
r
	​

cosϕcosλ−a
ϕ
	​

sinϕcosλ−a
λ
	​

sinλ
=a
r
	​

cosϕsinλ−a
ϕ
	​

sinϕsinλ+a
λ
	​

cosλ
=a
r
	​

sinϕ+a
ϕ
	​

cosϕ
	​



8. Gazebo 힘 적용 방식

중력력은 다음과 같이 계산된다.

𝐹
=
𝑚
⋅
𝑎
F=m⋅a


8.1 Gazebo API 적용
link->AddForce(ignition::math::Vector3d(Fx, Fy, Fz));

각 Update() 주기마다 중력력 계산 및 적용

Gazebo World 중력은 비활성화 권장

9. 시뮬레이션 효과

균일 중력 대비 현실적인 궤도 운동 구현

J2 및 고차 중력 섭동 재현 가능

실제 위성 비행 동역학과 높은 일치도 확보

10. 제한 사항

계산 복잡도는 차수 증가에 따라 급격히 증가

실시간 시뮬레이션에서는 차수 제한 필요

대기 항력, 태양복사압(SRP)은 별도 모델 필요

11. 향후 확장

ECI 좌표계 지원

대기 항력 모델 연동

SRP 및 제3체 중력 확장

ROS2 연동 궤도 검증 노드 추가

12. 참고

GOCO Gravity Field Models (NASA/JPL)

Montenbruck & Gill, Satellite Orbits

Vallado, Fundamentals of Astrodynamics and Applications
