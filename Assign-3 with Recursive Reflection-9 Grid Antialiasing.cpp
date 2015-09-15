/*
CSCI 420
Assignment 3 Raytracer

Name: Avinash Akka
USC ID: 3874-5774-01

*/

#include <pic.h>
#include <windows.h>
#include <stdlib.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <stdio.h>
#include <string>

#include <omp.h>

#define MAX_TRIANGLES 2000
#define MAX_SPHERES 10
#define MAX_LIGHTS 10
#define Recursive_Depth 2 //No of recursive reflection for the ray

char *filename=0;

//different display modes
#define MODE_DISPLAY 1
#define MODE_JPEG 2
int mode=MODE_JPEG;

double reflectColor[3];

//you may want to make these smaller for debugging purposes
#define WIDTH 640
#define HEIGHT 480

int CountR;

//the field of view of the camera
#define fov 60.0
#define M_PI  3.14

unsigned char buffer[HEIGHT][WIDTH][3];


struct Vertex
{
  double position[3];
  double Diffuse_Color[3];
  double Specular_Color[3];
  double normal[3];
  double shininess;
};

typedef struct _Triangle
{
  struct Vertex v[3];
} Triangle;


typedef struct _Sphere
{
  double position[3];
  double Diffuse_Color[3];
  double Specular_Color[3];
  double shininess;
  double radius;
} Sphere;

typedef struct _Light
{
  double position[3];
  double color[3];
} Light;

Triangle triangles[MAX_TRIANGLES];
Sphere spheres[MAX_SPHERES];
Light lights[MAX_LIGHTS];
double ambient_light[3];

int num_triangles=0;
int num_spheres=0;
int num_lights=0;

double Pixel_Width;
double Pixel_Height;
double Image_LeftEnd;
double Image_Bottom;

void plot_pixel_display(int x,int y,unsigned char r,unsigned char g,unsigned char b);
void plot_pixel_jpeg(int x,int y,unsigned char r,unsigned char g,unsigned char b);
void plot_pixel(int x,int y,unsigned char r,unsigned char g,unsigned char b);


//http://run.usc.edu/cs420-s15/lec15-ray-tracing/15-ray-tracing.pdf page 10
/* Find the border of our image window */
void Set_Window_Pixel() 
{
	double Aspect_Ratio = (double)WIDTH / (double)HEIGHT;

	//to find the Corner points of out image window
	double left = - Aspect_Ratio * tan(fov/2 * M_PI/180);
	
	double right = Aspect_Ratio * tan(fov/2 * M_PI/180);
	
	double top = tan((fov/2)*(M_PI/180));
	
	double bottom = - tan(fov/2 * M_PI/180);
	
	Pixel_Width = (right-left) / WIDTH;
	Pixel_Height = (top-bottom) / HEIGHT;
	Image_LeftEnd = left;
	Image_Bottom = bottom;
}

//Finds the Normal, was used in assignment 2 also
void Normalize(double *input, double *output) 
{
    // Find the mod which is sqrt(x*x + y*y + z*z)
	double magnitude = sqrt(input[0]*input[0] + input[1]*input[1] + input[2]*input[2]);
	if (magnitude == 0.0) 
		magnitude = 0.0001;
	output[0] = input[0] / magnitude;
	output[1] = input[1] / magnitude;
	output[2] = input[2] / magnitude;
}

//Finds the Dot Product, was used in assignment 2 also
double DotProduct(double *a, double * b) 
{
		double result;
		result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	    return result;
}

//Does a Vector Subtraction, was used in assignment 2 also
void VSubtraction (double *a, double *b, double *out) 
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

//Finds the Cross Product, was used in assignment 2 also
void CrossProduct(double *in1, double *in2, double *output) {
	output[0] = in1[1]*in2[2] - in2[1] * in1[2];
	output[1] = in2[0]*in1[2] - in1[0] * in2[2];
	output[2] = in1[0]*in2[1] - in1[1] * in2[0];
}

//To find the triangle area with 3 vector vertices
// As discussed in Class
double Triangle_Area (double *a, double *b, double *c) 
{
	double b_a[3],c_a[3],area[3],result;	
	VSubtraction(b,a, b_a);
	VSubtraction(c,a, c_a);
	CrossProduct(b_a, c_a, area);
	result = sqrt(area[0]*area[0] + area[1]*area[1] + area[2]*area[2])/2;
	return result;
}

//http://run.usc.edu/cs420-s15/lec16-geometric-queries/16-geometric-queries.pdf 
//Checks and finds the Triangle intersect
Triangle * Triangle_Intersect(double *direction, double * distance_out, double * translation) {
	Triangle * CurT = NULL;
	double Ray_[3];

	double Dirctn[3];
	Dirctn[0] = direction[0] - translation[0];
	Dirctn[1] = direction[1] - translation[1];
	Dirctn[2] = direction[2] - translation[2];
	Normalize(Dirctn, Dirctn);

	for(int x = 0; x < num_triangles; x++) 
	{
		double n[3];
		double p1_p0[3];	 VSubtraction(triangles[x].v[1].position,triangles[x].v[0].position,p1_p0);
		double p2_p0[3];	 VSubtraction(triangles[x].v[2].position,triangles[x].v[0].position,p2_p0);
		CrossProduct(p1_p0, p2_p0, n);
		Normalize(n, n);
		double n_dot_d = DotProduct(n,Dirctn);
		if (!(n_dot_d < 0.000001f && n_dot_d > 0.000001f)) { //If n_dot_p is zero, then this triangle is parallel to ray
			double origin_minus_p[3]; //Check here if erroring, it could be the other way around?
			origin_minus_p[0] = translation[0] - triangles[x].v[0].position[0];
			origin_minus_p[1] = translation[1] - triangles[x].v[0].position[1];
			origin_minus_p[2] = translation[2] - triangles[x].v[0].position[2];
			double o_minus_p_dot_n = DotProduct(origin_minus_p, n);
			double t = - o_minus_p_dot_n/n_dot_d;
			if (t>-.0000001f && t < .0000001f)
				t = 0.f;
			if (t>0.f) { //If the hit location is in front of us
				double hit[3];
				hit[0] = translation[0] + t * Dirctn[0];
				hit[1] = translation[1] + t * Dirctn[1];
				hit[2] = translation[2] + t * Dirctn[2];
				
				double p2_p1[3];	 VSubtraction(triangles[x].v[2].position,triangles[x].v[1].position,p2_p1);
				double p0_p2[3];	 VSubtraction(triangles[x].v[0].position,triangles[x].v[2].position,p0_p2);
				double hit_p1[3];	 VSubtraction(hit, triangles[x].v[0].position, hit_p1);
				double hit_p2[3];	 VSubtraction(hit, triangles[x].v[1].position, hit_p2);
				double hit_p3[3];	 VSubtraction(hit, triangles[x].v[2].position, hit_p3);
				 
				double cross_1[3];	 CrossProduct(p1_p0,hit_p1,cross_1);
				double cross_2[3];	 CrossProduct(p2_p1,hit_p2,cross_2);
				double cross_3[3];	 CrossProduct(p0_p2,hit_p3,cross_3);

				if (DotProduct(cross_1, n) >=0.0f && DotProduct(cross_2, n) >=0.0f && DotProduct(cross_3, n) >=0.0f) {
					if (t<*distance_out) {
						*distance_out = t;
						CurT = &triangles[x];
					}
				}
			}
		}
	}
	return CurT;
}

//http://run.usc.edu/cs420-s15/lec16-geometric-queries/16-geometric-queries.pdf 
//Checks and finds the Sphere intersect
Sphere* Sphere_Intersect(double * direction, double * distance_out, double * translation){
	Sphere * cur_sphere = NULL;
	double Ray_[3];

	double Dirctn[3];
	Dirctn[0] = direction[0] - translation[0];
	Dirctn[1] = direction[1] - translation[1];
	Dirctn[2] = direction[2] - translation[2];
	Normalize(Dirctn, Ray_);

	for(int x = 0; x < num_spheres; x++) 
	{
	   // Chapter Slides	
		double b = 2 * (Ray_[0]*(translation[0]-spheres[x].position[0]) + Ray_[1]*(translation[1]-spheres[x].position[1]) + Ray_[2]*(translation[2]-spheres[x].position[2]));
		double c = (translation[0]-spheres[x].position[0])*(translation[0]-spheres[x].position[0]) + (translation[1]-spheres[x].position[1])*(translation[1]-spheres[x].position[1]) + (translation[2]-spheres[x].position[2])*(translation[2]-spheres[x].position[2]) - spheres[x].radius*spheres[x].radius;
		double inside = b*b - 4 * c;//main condition to check

		if (inside >=0) { //check if (b*b - 4c) is negative, and abort if so
			double t0 = (-b + sqrt(inside))/2;
			double t1 = (-b - sqrt(inside))/2;
			if (t0>-0.0001f && t0 <= 0.0001f)
				t0 = 0.0f;
			if (t1>-0.0001f && t1 < 0.0001f)
				t1 = 0.0f; 
			/* If ray is cast from inside the sphere, it will hit sphere*/
			if (t0 > 0.f && t0 < *distance_out) { 
				*distance_out = t0;
				cur_sphere = &spheres[x];
			}
			if (t1 > 0.f && t1 < *distance_out) { 
				*distance_out = t1;
				cur_sphere = &spheres[x];
			}
		}
	}
	return cur_sphere;
}

bool Check_Shadows(double * source_transform, Light * destination_light) 
{
	double light_sphere_distance = sqrt((destination_light->position[0]-source_transform[0])*(destination_light->position[0]-source_transform[0]) + (destination_light->position[1]-source_transform[1])*(destination_light->position[1]-source_transform[1]) + (destination_light->position[2]-source_transform[2])*(destination_light->position[2]-source_transform[2]));
	double light_tri_distance = light_sphere_distance;

	if (Sphere_Intersect(destination_light->position, &light_sphere_distance, source_transform))
		return true;
	if(Triangle_Intersect(destination_light->position, &light_tri_distance, source_transform))
		return true;
	return false;
}

void cast_ray(double x, double y, double *color,int CountR) 
{
	
	
	if(CountR == Recursive_Depth)
		return;

	 color[0] = ambient_light[0];
	 color[1] = ambient_light[1];
	 color[2] = ambient_light[2];
	 
	double screen_position[3];
	screen_position[2] = -1;//Z is in negative direction
	screen_position[0] = Image_LeftEnd + Pixel_Width/2 + x*Pixel_Width;//loading X,Y values
	screen_position[1] = Image_Bottom + Pixel_Height/2 + y*Pixel_Height;
	
	double translation [3] = {0.0, 0.0, 0.0};

	double sphere_distance	= 400000000000;//for parallel rays
	double tri_distance		= 200000000000;//for parallel rays
	
	Sphere *hit_sphere = Sphere_Intersect(screen_position, &sphere_distance, translation);
	Triangle *hit_triangle = Triangle_Intersect(screen_position, &tri_distance, translation);

	double ray_Intersect_Point[3];
	double Ray_[3];

	Normalize(screen_position, Ray_);
	if (sphere_distance<tri_distance && hit_sphere) 
	{
		ray_Intersect_Point[0] = sphere_distance * Ray_[0];	
		ray_Intersect_Point[1] = sphere_distance * Ray_[1];	
		ray_Intersect_Point[2] = sphere_distance * Ray_[2];

		for (int x = 0; x < num_lights; x++ ) 
	{
			if (!Check_Shadows(ray_Intersect_Point, &lights[x])) 
		{//If not in shadow
						
 			double normal[3];
			normal[0] = ray_Intersect_Point[0] - hit_sphere->position[0];
			normal[1] = ray_Intersect_Point[1] - hit_sphere->position[1];
			normal[2] = ray_Intersect_Point[2] - hit_sphere->position[2];
			Normalize(normal, normal);

			double view_vector[3];
			view_vector[0] = -ray_Intersect_Point[0];	
			view_vector[1] = -ray_Intersect_Point[1];	
			view_vector[2] = -ray_Intersect_Point[2];	
			Normalize(view_vector, view_vector);

			double light_vector[3];
			light_vector[0] = lights->position[0] - ray_Intersect_Point[0];
			light_vector[1] = lights->position[1] - ray_Intersect_Point[1];
			light_vector[2] = lights->position[2] - ray_Intersect_Point[2];
			Normalize(light_vector, light_vector);

			double LxN = DotProduct(light_vector, normal);
			double reflected_vector[3]; // r = 2 * LxN * n - l
	
			reflected_vector[0] =2 * LxN * normal[0] - light_vector[0];
			reflected_vector[1] =2 * LxN * normal[1] - light_vector[1];
			reflected_vector[2] =2 * LxN * normal[2] - light_vector[2];
			Normalize(reflected_vector, reflected_vector);

			double RxV = DotProduct(reflected_vector, view_vector);
			if (LxN < 0)
				LxN = 0;
			if (RxV < 0)
				RxV = 0;

			color[0] += lights->color[0] * (hit_sphere->Diffuse_Color[0] * (LxN) + hit_sphere->Specular_Color[0] * pow(RxV,hit_sphere->shininess));
			color[1] += lights->color[0] * (hit_sphere->Diffuse_Color[1] * (LxN) + hit_sphere->Specular_Color[1] * pow(RxV,hit_sphere->shininess));
			color[2] += lights->color[0] * (hit_sphere->Diffuse_Color[2] * (LxN) + hit_sphere->Specular_Color[2] * pow(RxV,hit_sphere->shininess));
			
			/*if(CountR == Recursive_Depth)
				return;
			reflectColor[0] = color[0];
			reflectColor[1] = color[1];
			reflectColor[2] = color[2];*/
   
			cast_ray(x,y,reflectColor,CountR+1);

			//color[0] =((1 - hit_sphere->Specular_Color[0]) * color[0] ) + hit_sphere->Specular_Color[0] * reflectColor[0];
			//color[1] =((1 - hit_sphere->Specular_Color[1]) * color[1] ) + hit_sphere->Specular_Color[1] * reflectColor[1];
  			//color[2] =((1 - hit_sphere->Specular_Color[2]) * color[2] ) + hit_sphere->Specular_Color[2] * reflectColor[2];


			}
		}
	}	
	
	else if (hit_triangle) 
	{
		ray_Intersect_Point[0] = tri_distance * Ray_[0];	
		ray_Intersect_Point[1] = tri_distance * Ray_[1];	
		ray_Intersect_Point[2] = tri_distance * Ray_[2];
		
		for (int x = 0; x < num_lights; x++ ) 
		{
			if (!Check_Shadows(ray_Intersect_Point, &lights[x])) 
		  {
			
			double Diffuse_Color[3];
			double Specular_Color[3];
			double shininess;
	
			double normal[3];
			//Find the Triangle area
			double Area = Triangle_Area(hit_triangle->v[0].position, hit_triangle->v[1].position, hit_triangle->v[2].position);

			double P0value = Triangle_Area(hit_triangle->v[1].position, hit_triangle->v[2].position, ray_Intersect_Point) / Area;
			double P1value = Triangle_Area(hit_triangle->v[2].position, hit_triangle->v[0].position, ray_Intersect_Point)/ Area;
			double P2value = Triangle_Area(hit_triangle->v[0].position, hit_triangle->v[1].position, ray_Intersect_Point)/ Area;

			normal[0] = P0value * hit_triangle->v[0].normal[0] + P1value * hit_triangle->v[1].normal[0] + P2value * hit_triangle->v[2].normal[0];
			normal[1] = P0value * hit_triangle->v[0].normal[1] + P1value * hit_triangle->v[1].normal[1] + P2value * hit_triangle->v[2].normal[1];
			normal[2] = P0value * hit_triangle->v[0].normal[2] + P1value * hit_triangle->v[1].normal[2] + P2value * hit_triangle->v[2].normal[2];

			Normalize(normal, normal);

			//http://run.usc.edu/cs420-s15/recitations/hw3Hints.pdf
			//As explained in class slides and hints

			Diffuse_Color[0] = P0value * hit_triangle->v[0].Diffuse_Color[0] + P1value * hit_triangle->v[1].Diffuse_Color[0] + P2value * hit_triangle->v[2].Diffuse_Color[0];
			Diffuse_Color[1] = P0value * hit_triangle->v[0].Diffuse_Color[1] + P1value * hit_triangle->v[1].Diffuse_Color[1] + P2value * hit_triangle->v[2].Diffuse_Color[1];
			Diffuse_Color[2] = P0value * hit_triangle->v[0].Diffuse_Color[2] + P1value * hit_triangle->v[1].Diffuse_Color[2] + P2value * hit_triangle->v[2].Diffuse_Color[2];

			Specular_Color[0] = P0value * hit_triangle->v[0].Specular_Color[0] + P1value * hit_triangle->v[1].Specular_Color[0] +P2value * hit_triangle->v[2].Specular_Color[0];
			Specular_Color[1] = P0value * hit_triangle->v[0].Specular_Color[1] + P1value * hit_triangle->v[1].Specular_Color[1] +P2value * hit_triangle->v[2].Specular_Color[1];
			Specular_Color[2] = P0value * hit_triangle->v[0].Specular_Color[2] + P1value * hit_triangle->v[1].Specular_Color[2] +P2value * hit_triangle->v[2].Specular_Color[2];

			shininess = P0value * hit_triangle->v[0].shininess + P1value * hit_triangle->v[1].shininess + P2value * hit_triangle->v[2].shininess;
	
			double view_vector[3];
			view_vector[0] = -ray_Intersect_Point[0];	
			view_vector[1] = -ray_Intersect_Point[1];	
			view_vector[2] = -ray_Intersect_Point[2];	
			Normalize(view_vector, view_vector);

			double light_vector[3];
			light_vector[0] = lights->position[0] - ray_Intersect_Point[0];
			light_vector[1] = lights->position[1] - ray_Intersect_Point[1];
			light_vector[2] = lights->position[2] - ray_Intersect_Point[2];
			Normalize(light_vector, light_vector);
	
			double LxN = DotProduct(light_vector, normal);
			double reflected_vector[3]; // r = 2 * LxN * n - l
			reflected_vector[0] =2 * LxN * normal[0] - light_vector[0];
			reflected_vector[1] =2 * LxN * normal[1] - light_vector[1];
			reflected_vector[2] =2 * LxN * normal[2] - light_vector[2];
			Normalize(reflected_vector, reflected_vector);
	
			double RxV = DotProduct(reflected_vector, view_vector);
			if (LxN < 0)
				LxN = 0;
			if (RxV < 0)
				RxV = 0;

	         //(1 - Specular_Color[]) * reflectColor[] + Specular_Color[] * color[]
			color[0] += lights->color[0] * (Diffuse_Color[0] * (LxN) + Specular_Color[0] * pow(RxV,shininess));
			color[1] += lights->color[1] * (Diffuse_Color[1] * (LxN) + Specular_Color[1] * pow(RxV,shininess));
			color[2] += lights->color[2] * (Diffuse_Color[2] * (LxN) + Specular_Color[2] * pow(RxV,shininess));
			 
			/*if(CountR == Recursive_Depth)
				return;
			reflectColor[0] = color[0];
			reflectColor[1] = color[1];
			reflectColor[2] = color[2];*/
   
			cast_ray(x,y,reflectColor,CountR+1);

			//color[0] =((1 - Specular_Color[0]) * color[0] )+ Specular_Color[0] * reflectColor[0];
			//color[1] =((1 - Specular_Color[1]) * color[1] )+ Specular_Color[1] * reflectColor[1];
  			//color[2] =((1 - Specular_Color[2]) * color[2] )+ Specular_Color[2] * reflectColor[2];

			}
		}
	} 
	
	else 
	 {//missed
		color[0] = 1.0f; color[1] = 1.0f; color[2] = 1.0f;
	 }
	 
	  
}

void cast_aa_ray(int x, int y) 
{
	
	double color1[3],color2[3],color3[3],color4[3],color5[3],color6[3],color7[3],color8[3],color9[3];
	
	
	/*Anti-Aliasing by super sampling for each pixel using grid algorithm method
	---------
    | . . . |
	| . . . |
	| . . . |
	---------
	*/
	
	cast_ray(x+.25, y+.25, color1,CountR);
	cast_ray(x+.5, y+.25, color2,CountR);
	cast_ray(x+.75, y+.25, color3,CountR);
	cast_ray(x+.25, y+.5, color4,CountR);
	cast_ray(x+.5, y+.5, color5,CountR);
	cast_ray(x+.75, y+.5, color6,CountR);
	cast_ray(x+.25, y+.75, color7,CountR);
	cast_ray(x+.5, y+.75, color8,CountR);
	cast_ray(x+.75, y+.75, color9,CountR);
	
	double color[3];
	
	color[0] = (color1[0]+color2[0]+color3[0]+color4[0]+color5[0]+color6[0]+color7[0]+color8[0]+color9[0]) / 9;//averaging 9 value
	color[1] = (color1[1]+color2[1]+color3[1]+color4[1]+color5[0]+color6[0]+color7[0]+color8[0]+color9[1]) / 9;
	color[2] = (color1[2]+color2[2]+color3[2]+color4[2]+color5[0]+color6[0]+color7[0]+color8[0]+color9[2]) / 9;
	
	  
	if(color[0] > 1)
		color[0] = 1;
	else if (color[0] < 0)
		color[0] = 0;
	color[0] *= 255;

	if(color[1] > 1)
		color[1] = 1;
	else if (color[1] < 0)
		color[1] = 0;
	color[1] *= 255;

	if(color[2] > 1)
		color[2] = 1;
	else if (color[2] < 0)
		color[2] = 0;
	color[2] *= 255;

	plot_pixel_jpeg(x,y,color[0],color[1],color[2]);

}

void save_jpg()
{
  Pic *in = NULL;

  in = pic_alloc(640, 480, 3, NULL);
  printf("Saving JPEG file: %s\n", filename);

  memcpy(in->pix,buffer,3*WIDTH*HEIGHT);
  if (jpeg_write(filename, in))
    printf("File saved Successfully\n");
  else
    printf("Error in Saving\n");

  pic_free(in);      

}

void draw_scene()
{  
	glutSwapBuffers();

	 for(int x=0; x<WIDTH; x++) 
	 {
		glPointSize(2.0);  
		glBegin(GL_POINTS);
		for(int y=0;y < HEIGHT;y++)
		{
			cast_aa_ray(x,y);
			plot_pixel(x,y,buffer[HEIGHT-y-1][x][0],buffer[HEIGHT-y-1][x][1],buffer[HEIGHT-y-1][x][2]);
		}
		glEnd();
		glFlush();
	  }
   }

void render_scene() 
{
    #pragma omp for nowait schedule(dynamic,10)
	for(int x=0; x<WIDTH; x++)
	  {
		for(int y=0;y < HEIGHT;y++)
		{
			cast_aa_ray(x,y);
		}
	  }

}

void plot_pixel_display(int x,int y,unsigned char r,unsigned char g,unsigned char b)
{
  glColor3f(((double)r)/256.f,((double)g)/256.f,((double)b)/256.f);
  glVertex2f(x,y);
}
void plot_pixel_jpeg(int x,int y,unsigned char r,unsigned char g,unsigned char b)
{
  buffer[HEIGHT-y-1][x][0]=r;
  buffer[HEIGHT-y-1][x][1]=g;
  buffer[HEIGHT-y-1][x][2]=b;
}

void plot_pixel(int x,int y,unsigned char r,unsigned char g, unsigned char b)
{
  plot_pixel_display(x,y,r,g,b);
  if(mode == MODE_JPEG)
      plot_pixel_jpeg(x,y,r,g,b);
}


void parse_check(char *expected,char *found)
{
  if(stricmp(expected,found))
    {
      char error[100];
      printf("Expected '%s ' found '%s '\n",expected,found);
      printf("Parse error, abnormal abortion\n");
      exit(0);
    }

}
void parse_doubles(FILE*file, char *check, double p[3])
{
  char str[100];
  fscanf(file,"%s",str);
  parse_check(check,str);
  fscanf(file,"%lf %lf %lf",&p[0],&p[1],&p[2]);
  printf("%s %lf %lf %lf\n",check,p[0],p[1],p[2]);
}
void parse_rad(FILE*file,double *r)
{
  char str[100];
  fscanf(file,"%s",str);
  parse_check("rad:",str);
  fscanf(file,"%lf",r);
  printf("rad: %f\n",*r);
}
void parse_shi(FILE*file,double *shi)
{
  char s[100];
  fscanf(file,"%s",s);
  parse_check("shi:",s);
  fscanf(file,"%lf",shi);
  printf("shi: %f\n",*shi);
}
int loadScene(char *argv)
{
  FILE *file = fopen(argv,"r");
  int number_of_objects;
  char type[50];
  int i;
  Triangle t;
  Sphere s;
  Light l;
  fscanf(file,"%i",&number_of_objects);

  printf("number of objects: %i\n",number_of_objects);
  char str[200];

  parse_doubles(file,"amb:",ambient_light);

  for(i=0;i < number_of_objects;i++)
    {
      fscanf(file,"%s\n",type);
      printf("%s\n",type);
      if(stricmp(type,"triangle")==0)
	{

	  printf("found triangle\n");
	  int j;

	  for(j=0;j < 3;j++)
	    {
	      parse_doubles(file,"pos:",t.v[j].position);
	      parse_doubles(file,"nor:",t.v[j].normal);
	      parse_doubles(file,"dif:",t.v[j].Diffuse_Color);
	      parse_doubles(file,"spe:",t.v[j].Specular_Color);
	      parse_shi(file,&t.v[j].shininess);
	    }

	  if(num_triangles == MAX_TRIANGLES)
	    {
	      printf("too many triangles, you should increase MAX_TRIANGLES!\n");
	      exit(0);
	    }
	  triangles[num_triangles++] = t;
	}
      else if(stricmp(type,"sphere")==0)
	{
	  printf("found sphere\n");

	  parse_doubles(file,"pos:",s.position);
	  parse_rad(file,&s.radius);
	  parse_doubles(file,"dif:",s.Diffuse_Color);
	  parse_doubles(file,"spe:",s.Specular_Color);
	  parse_shi(file,&s.shininess);

	  if(num_spheres == MAX_SPHERES)
	    {
	      printf("too many spheres, you should increase MAX_SPHERES!\n");
	      exit(0);
	    }
	  spheres[num_spheres++] = s;
	}
      else if(stricmp(type,"light")==0)
	{
	  printf("found light\n");
	  parse_doubles(file,"pos:",l.position);
	  parse_doubles(file,"col:",l.color);

	  if(num_lights == MAX_LIGHTS)
	    {
	      printf("too many lights, you should increase MAX_LIGHTS!\n");
	      exit(0);
	    }
	  lights[num_lights++] = l;
	}
      else
	{
	  printf("unknown type in scene description:\n%s\n",type);
	  exit(0);
	}
    }
  return 0;
}
void display()
{

}
void init()
{
  glMatrixMode(GL_PROJECTION);
  glOrtho(0,WIDTH,0,HEIGHT,1,-1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glClearColor(0,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT);
  Set_Window_Pixel();
}
void idle()
{
	
	//hack to make it only draw once
  static int once=0;
  if(!once)
  {
	     draw_scene();
      if(mode == MODE_JPEG)
		save_jpg();
    }
  once=1;
}

int main (int argc, char ** argv)
{
  
  if (argc<2 || argc > 3)
  {  
    printf ("usage: %s <scenefile> [jpegname]\n", argv[0]);
    exit(0);
  }
  if(argc == 3)
    {
      mode = MODE_JPEG;
      filename = argv[2];
    }
  else if(argc == 2)
    mode = MODE_DISPLAY;

  glutInit(&argc,argv);
  loadScene(argv[1]);

  glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
  glutInitWindowPosition(0,0);
  glutInitWindowSize(WIDTH,HEIGHT);
  int window = glutCreateWindow("Ray Tracer");
  glutDisplayFunc(display);
  glutIdleFunc(idle);
  init();
  render_scene();//To use multi threading
  glutMainLoop();
  
}

