!
!-------------------------- Default Units for Model ---------------------------!
!
!
defaults units  &
   length = mm  &
   angle = deg  &
   force = newton  &
   mass = kg  &
   time = sec
!
defaults units  &
   coordinate_system_type = cartesian  &
   orientation_type = body313
!
!------------------------ Default Attributes for Model ------------------------!
!
!
defaults attributes  &
   inheritance = bottom_up  &
   icon_visibility = on  &
   grid_visibility = off  &
   size_of_icons = 50.0  &
   spacing_for_grid = 1000.0
!
!------------------------------ Adams/View Model ------------------------------!
!
!
model create  &
   model_name = sus_2d
!
view erase
!
!--------------------------------- Materials ----------------------------------!
!
!
material create  &
   material_name = .sus_2d.steel  &
   youngs_modulus = 2.07E+005  &
   poissons_ratio = 0.29  &
   density = 7.801E-006
!
!-------------------------------- Rigid Parts ---------------------------------!
!
! Create parts and their dependent markers and graphics
!
!----------------------------------- ground -----------------------------------!
!
!
! ****** Ground Part ******
!
defaults model  &
   part_name = ground
!
defaults coordinate_system  &
   default_coordinate_system = .sus_2d.ground
!
part create rigid_body mass_properties  &
   part_name = .sus_2d.ground  &
   material_type = .sus_2d.steel
!
part attributes  &
   part_name = .sus_2d.ground  &
   name_visibility = off
!
!-------------------------- suspension 2d ----------------------------!

!-------------------------- points ----------------------------!

point create point = .sus_2d.ground.p_lower_wishbone_chassis &
	location = 500.0, 150.0, 0.0 

point create point = .sus_2d.ground.p_upper_wishbone_chassis &
	location = 500.0, 510.0, 0.0 

point create point = .sus_2d.ground.p_lower_wishbone_ball_joint &
	location = 150.0, 150.0, 0.0 

point create point = .sus_2d.ground.p_upper_wishbone_ball_joint &
	location = 200.0, 600.0, 0.0
	
point create point = .sus_2d.ground.p_wheel_centre &
	location = 0.0, 300.0, 0.0

point create point = .sus_2d.ground.p_wheel_base &
	location = 0.0, 0.0, 0.0
	
!-------------------------- upper wishbone ----------------------------!

part create rigid_body name_and_position &
	part_name = .sus_2d.upper_wishbone
	
marker create marker = .sus_2d.upper_wishbone.m_upper_wishbone_cm &
    location = 350.0, 555.0, 0.0 &
    orientation = 0.0, 0.0, 0.0
	
part modify rigid mass_properties  &
    part_name = .sus_2d.upper_wishbone  &
    mass = 1.0  &
    center_of_mass_marker = .sus_2d.upper_wishbone.m_upper_wishbone_cm  &
    ixx = 3.0e3  &
    iyy = 3.0e3  &
    izz = 3.0e3

part attributes &
	part_name = .sus_2d.upper_wishbone &
	color = RED &
	name_vis = off

marker create &
	marker = .sus_2d.upper_wishbone.m_upper_wishbone_chassis &
!	location = 500.0, 510.0, 0.0
    location =(LOC_RELATIVE_TO({0,0,0}, p_upper_wishbone_chassis)) &
    orientation = 0.0, 0.0,0.0

marker create marker = .sus_2d.upper_wishbone.m_upper_ball_joint &
    location =(LOC_RELATIVE_TO({0,0,0}, p_upper_wishbone_ball_joint)) &
    orientation = 0.0, 0.0, 0.0

geometry create shape link &
    link_name = .sus_2d.upper_wishbone.g_upper_wishbone &
    width =(30.0 mm) &
    depth =(15.0 mm) &
    i_marker = .sus_2d.upper_wishbone.m_upper_wishbone_chassis &
    j_marker = .sus_2d.upper_wishbone.m_upper_ball_joint

	
!-------------------------- lower wishbone ----------------------------!

part create rigid_body name_and_position &
	part_name = .sus_2d.lower_wishbone
	
marker create marker = .sus_2d.lower_wishbone.m_lower_wishbone_cm &
    location = 325.0, 150.0, 0.0 &
    orientation = 0.0, 0.0, 0.0
	
part modify rigid mass_properties  &
    part_name = .sus_2d.lower_wishbone  &
    mass = 5.0  &
    center_of_mass_marker = .sus_2d.lower_wishbone.m_lower_wishbone_cm  &
    ixx = 5.0e3  &
    iyy = 5.0e3  &
    izz = 5.0e3

part attributes &
	part_name = .sus_2d.lower_wishbone &
	color = BLUE &
	name_vis = off

marker create &
	marker = .sus_2d.lower_wishbone.m_lower_wishbone_chassis &
!	location = 500.0, 510.0, 0.0
    location =(LOC_RELATIVE_TO({0,0,0}, p_lower_wishbone_chassis)) &
    orientation = 0.0, 0.0,0.0

marker create marker = .sus_2d.lower_wishbone.m_lower_ball_joint &
    location =(LOC_RELATIVE_TO({0,0,0}, p_lower_wishbone_ball_joint)) &
    orientation = 0.0, 0.0, 0.0

geometry create shape link &
    link_name = .sus_2d.lower_wishbone.g_lower_wishbone &
    width =(30.0 mm) &
    depth =(15.0 mm) &
    i_marker = .sus_2d.lower_wishbone.m_lower_wishbone_chassis &
    j_marker = .sus_2d.lower_wishbone.m_lower_ball_joint

!-------------------------- upright ----------------------------!

part create rigid_body name_and_position &
	part_name = .sus_2d.upright
	
marker create marker = .sus_2d.upright.m_upright_cm &
    location = 0.0, 300.0, 0.0 &
    orientation = 0.0, 0.0, 0.0
	
part modify rigid mass_properties  &
    part_name = .sus_2d.upright  &
    mass = 50.0  &
    center_of_mass_marker = .sus_2d.upright.m_upright_cm  &
    ixx = 1.0e6  &
    iyy = 1.0e6  &
    izz = 1.0e6

part attributes &
	part_name = .sus_2d.upright &
	color = yellow &
	name_vis = off

marker create &
	marker = .sus_2d.upright.m_upright_upper_ball_joint &
!	location = 500.0, 510.0, 0.0
    location =(LOC_RELATIVE_TO({0,0,0}, p_upper_wishbone_ball_joint)) &
    orientation = 0.0, 0.0,0.0

marker create marker = .sus_2d.upright.m_upright_lower_ball_joint &
    location =(LOC_RELATIVE_TO({0,0,0}, p_lower_wishbone_ball_joint)) &
    orientation = 0.0, 0.0, 0.0

geometry create shape link &
    link_name = .sus_2d.upright.g_upright &
    width =(30.0 mm) &
    depth =(15.0 mm) &
    i_marker = .sus_2d.upright.m_upright_upper_ball_joint &
    j_marker = .sus_2d.upright.m_upright_lower_ball_joint

marker create marker = .sus_2d.upright.m_upright_graphic &
    location = 170.0, 300.0, 0.0 &
    orientation = 0.0, 0.0, 0.0

geometry create shape link &
    link_name = .sus_2d.upright.g_upright_2 &
    width =(30.0 mm) &
    depth =(15.0 mm) &
    i_marker = .sus_2d.upright.m_upright_graphic &
    j_marker = .sus_2d.upright.m_upright_cm

marker create &
	marker=.sus_2d.upright.m_upright_wheel_graphic &
    location=(LOC_RELATIVE_TO({0,0,0}, p_wheel_centre)) &
    orientation=270.0,90.0,180.0
	
geometry create shape cylinder &
    cylinder_name=.sus_2d.upright.g_wheel &
    length=(200.0mm) &
    radius=(300.0mm) &
    angle=270.0d &
    center_marker=.sus_2d.upright.m_upright_wheel_graphic

!-------------------------- constraints ----------------------------!

marker create marker=.sus_2d.upper_wishbone.m_upper_wishbone_chassis_joint &
    location=(LOC_RELATIVE_TO({0,0,0}, p_upper_wishbone_chassis)) &
    orientation=0.0, 0.0, 0.0
	
marker create marker=.sus_2d.ground.m_upper_wishbone_chassis_joint_ground &
    location=(LOC_RELATIVE_TO({0,0,0}, p_upper_wishbone_chassis)) &
    orientation=0.0, 0.0, 0.0
	
constraint create joint Revolute &
    joint_name=.sus_2d.j_upper_wishbone_rev &
    i_marker_name=.sus_2d.upper_wishbone.m_upper_wishbone_chassis_joint &
    j_marker_name=.sus_2d.ground.m_upper_wishbone_chassis_joint_ground 

marker create marker=.sus_2d.lower_wishbone.m_lower_wishbone_chassis_joint &
    location=(LOC_RELATIVE_TO({0,0,0}, p_lower_wishbone_chassis)) &
    orientation=0.0, 0.0, 0.0
	
marker create marker=.sus_2d.ground.m_lower_wishbone_chassis_joint_ground &
    location=(LOC_RELATIVE_TO({0,0,0}, p_lower_wishbone_chassis)) &
    orientation=0.0, 0.0, 0.0
	
constraint create joint Revolute &
    joint_name=.sus_2d.j_lower_wishbone_rev &
    i_marker_name=.sus_2d.lower_wishbone.m_lower_wishbone_chassis_joint &
    j_marker_name=.sus_2d.ground.m_lower_wishbone_chassis_joint_ground 

constraint create joint Spherical &
    joint_name=.sus_2d.j_upper_ball_joint_rev &
    i_marker_name=.sus_2d.upright.m_upright_upper_ball_joint &
    j_marker_name=.sus_2d.upper_wishbone.m_upper_ball_joint 

constraint create joint Cylindrical &
    joint_name=.sus_2d.j_lower_ball_joint_rev &
    i_marker_name=.sus_2d.upright.m_upright_lower_ball_joint &
    j_marker_name=.sus_2d.lower_wishbone.m_lower_ball_joint 


constraint create motion motion_name=.sus_2d.MOTION_1 &
           joint=.sus_2d.j_upper_wishbone_rev &
           type=rotational &
           time_derivative=displacement &
           function="30.0d * sin(360D * time)"











!-------------------------- YOUR MODEL ENDS HERE ------------------------------!
!
!----------------------------------- Forces -----------------------------------!
!
!
!---------------------------------- Accgrav -----------------------------------!
!
!
force create body gravitational  &
   gravity_field_name = gravity  &
   x_component_gravity = 0.0  &
   y_component_gravity = 0.0  &
   z_component_gravity = -9806.65
!
!----------------------------- Analysis settings ------------------------------!
!
!
!--------------------------- Expression definitions ---------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = ground
!
material modify  &
   material_name = .sus_2d.steel  &
   youngs_modulus = (2.07E+011(Newton/meter**2))  &
   density = (7801.0(kg/meter**3))
!
model display  &
   model_name = sus_2d
