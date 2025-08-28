#include "Viewer.hpp"
#include "GL/Quad.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

static Youth::GL::Quad gQuad;

namespace
{
    static const char *kTexVert = R"(#version 330 core
    layout(location=0) in vec2 aPos;
    layout(location=1) in vec2 aUV;
    out vec2 vUV;
    void main(){
        gl_Position = vec4(aPos,0.0,1.0);
        vUV = aUV;
    }
    )";

    static const char *kTexFrag = R"(#version 330 core
    in vec2 vUV;
    out vec4 FragColor;
    uniform sampler2D uTex;
    uniform bool uIsDepth;
    void main(){
        vec4 c = texture(uTex, vUV);
        if(uIsDepth){
            // 단일 채널 depth → 회색톤 시각화
            float d = c.r;
            FragColor = vec4(d,d,d,1.0);
        }else{
            FragColor = c;
        }
    }
    )";

    static const char *kVert = R"(#version 330 core
    layout(location = 0) in vec3 aPos;
    layout(location = 1) in vec3 aColor;
    out vec3 vColor;
    void main() {
        gl_Position = vec4(aPos, 1.0);
        vColor = aColor;
    }
    )";

    static const char *kFrag = R"(#version 330 core
    in vec3 vColor;
    out vec4 FragColor;
    void main() {
        FragColor = vec4(vColor, 1.0);
    }
    )";

    GLuint compile(GLenum type, const char *src)
    {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        GLint ok = GL_FALSE;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok)
        {
            GLint len = 0;
            glGetShaderiv(s, GL_INFO_LOG_LENGTH, &len);
            std::string log(len, '\0');
            glGetShaderInfoLog(s, len, nullptr, log.data());
            glDeleteShader(s);
            throw std::runtime_error("Shader compile error: " + log);
        }
        return s;
    }

    GLuint link(GLuint vs, GLuint fs)
    {
        GLuint p = glCreateProgram();
        glAttachShader(p, vs);
        glAttachShader(p, fs);
        glLinkProgram(p);
        GLint ok = GL_FALSE;
        glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok)
        {
            GLint len = 0;
            glGetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
            std::string log(len, '\0');
            glGetProgramInfoLog(p, len, nullptr, log.data());
            glDeleteProgram(p);
            throw std::runtime_error("Program link error: " + log);
        }
        return p;
    }

    void glfwErrorCallback(int code, const char *desc)
    {
        std::fprintf(stderr, "[GLFW] Error %d: %s\n", code, desc);
    }

} // namespace

namespace Youth
{

    Viewer::Viewer(const Config &cfg) : cfg_(cfg)
    {
        if (!initGLFW())
            return;
        if (!initGLEW())
            return;

        std::fprintf(stdout, "GL Vendor  : %s\n", glGetString(GL_VENDOR));
        std::fprintf(stdout, "GL Renderer: %s\n", glGetString(GL_RENDERER));
        std::fprintf(stdout, "GL Version : %s\n", glGetString(GL_VERSION));

        const std::array<float, 18> verts = {
            0.0f,
            0.5f,
            0.0f,
            1.0f,
            0.2f,
            0.2f,
            -0.5f,
            -0.5f,
            0.0f,
            0.2f,
            1.0f,
            0.2f,
            0.5f,
            -0.5f,
            0.0f,
            0.2f,
            0.2f,
            1.0f,
        };

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);

        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts.data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));

        GLuint vs = 0, fs = 0;
        try
        {
            vs = compile(GL_VERTEX_SHADER, kVert);
            fs = compile(GL_FRAGMENT_SHADER, kFrag);
            shader_prog_ = link(vs, fs);
        }
        catch (...)
        {
            if (vs)
                glDeleteShader(vs);
            if (fs)
                glDeleteShader(fs);
            throw;
        }
        glDeleteShader(vs);
        glDeleteShader(fs);

        {
            GLuint vs = 0, fs = 0;
            vs = compile(GL_VERTEX_SHADER, kTexVert);
            fs = compile(GL_FRAGMENT_SHADER, kTexFrag);
            tex_prog_ = link(vs, fs);
            glDeleteShader(vs);
            glDeleteShader(fs);
        }

        glBindVertexArray(0);

        if (!gQuad.init())
        {
            throw std::runtime_error("Quad init failed");
        }
    }

    Viewer::~Viewer()
    {
        if (shader_prog_)
            glDeleteProgram(shader_prog_);
        if (vbo_)
            glDeleteBuffers(1, &vbo_);
        if (vao_)
            glDeleteVertexArrays(1, &vao_);
        if (window_)
        {
            glfwDestroyWindow(window_);
            glfwTerminate();
            window_ = nullptr;
        }
        if (tex_prog_)
            glDeleteProgram(tex_prog_);
    }

    bool Viewer::initGLFW()
    {
        glfwSetErrorCallback(glfwErrorCallback);
        if (!glfwInit())
        {
            std::fprintf(stderr, "[Viewer] Failed to init GLFW\n");
            return false;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

        window_ = glfwCreateWindow(cfg_.width, cfg_.height, cfg_.title.c_str(), nullptr, nullptr);
        if (!window_)
        {
            std::fprintf(stderr, "[Viewer] Failed to create window\n");
            glfwTerminate();
            return false;
        }
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(cfg_.vsync ? 1 : 0);
        return true;
    }

    bool Viewer::initGLEW()
    {
        glewExperimental = GL_TRUE;
        GLenum err = glewInit();
        glGetError(); // clear benign error
        if (err != GLEW_OK)
        {
            std::fprintf(stderr, "[Viewer] Failed to init GLEW: %s\n", glewGetErrorString(err));
            return false;
        }
        return true;
    }

    void Viewer::drawTriangle()
    {
        glUseProgram(shader_prog_);
        glBindVertexArray(vao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glUseProgram(0);
    }

    void Viewer::poll()
    {
        glfwPollEvents();
        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        {
            glfwSetWindowShouldClose(window_, GLFW_TRUE);
        }
    }

    void Viewer::run()
    {
        while (!glfwWindowShouldClose(window_))
        {
            int fbw = 0, fbh = 0;
            glfwGetFramebufferSize(window_, &fbw, &fbh);
            glViewport(0, 0, fbw, fbh);

            glClearColor(cfg_.clear.r, cfg_.clear.g, cfg_.clear.b, cfg_.clear.a);
            glClear(GL_COLOR_BUFFER_BIT);

            if (draw_callback_)
            {
                draw_callback_();
            }
            else
            {
                drawTriangle();
            }

            glfwSwapBuffers(window_);
            poll();
        }
    }

    void Viewer::drawTexture2D(unsigned tex, int w, int h, bool isDepth)
    {
        glUseProgram(tex_prog_);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex);
        glUniform1i(glGetUniformLocation(tex_prog_, "uTex"), 0);
        glUniform1i(glGetUniformLocation(tex_prog_, "uIsDepth"), isDepth ? 1 : 0);

        gQuad.render();

        glBindTexture(GL_TEXTURE_2D, 0);
        glUseProgram(0);
    }

} // namespace Youth