#include "Shader.h"



class Irradiance{
    
private:
    
    Shader * irradiance, * prefilter, * brdfShader ;
    unsigned int vao, cubemap, FBO, RBO, irradiance_map, irradiance_shader, prefiltered_env_map, prefilter_shader, brdf_shader, brdfLUTTexture, num_progress_bars ;

public:
    
    Irradiance() ;
    void set(unsigned int vao, unsigned int cubemap, unsigned int FBO, unsigned int RBO ) ;
    void create_irradiance_map() ;
    void fill_irradiance_map() ;
    void create_prefiltered_environment_map() ;
    void fill_prefiltered_environment_map() ;
    void generate_brdf_LUT() ;
    void renderQuad() ;
    void shade() ;
    void activate_irradiance() ;
    void create_brdf_integration_map() ;
    ~Irradiance() ;
    
} ;

