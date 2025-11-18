# Dissfusion  IDL lecture 23 11/17

VAE take one giant step from noise to image -> blurry

Diffusion model break this step into to many tiny steps

## Definintion

what is diffusion processs?

### Forward process

    gradual noise addition
    马卡洛夫链

### Reverse Process

    Denoise
    ()

## How to add noise

Gaussian noise

每个diffusion step 逐渐添加

    input: noisey image timestep
    output: predicted noise

## Training obj

    MSE Error (added noise and predicted during forward process)
    公式： /alpha scaling

    Reconstruction

## Training and sampling

    timestep
    alpha 
    noise

    sampling 是做什么？

## DDIM

    DDPM Cahllenges

    1000 timessteps requly
    slow generation

    DDIM 

    can skip time steps intelligently

### Randam sampling


    x0:
    noise schedule /mu
    denoise function /sigma
    estimated noise /eqslion


    non-Markovian process
    fewer tiemsteps
    deterinistic sampling

    3 components: predicted x0, , 

## stochastics differential equations (SDE) and score matching

    DDPM fixed timesteps and nosie schedule

    DDIM: continuous time and smooth noise function (/beat(t))

### Modeling diffusion

    SDE

    Probability Flow ODEs -> Deterministics Sampling

    exact reconstruction

    Unified frmaeworks


ODE vs. SDE

### Forward Diffusion

    drift term pull torward mode
    diffusion term inject noise

### score function

    no need for computing normalizing const Z

    model the score function 
    /delta_xlogp(x)
    point towards high probability regions

### donoising score matching

    backward

## CFG Classifier-free Guidance

    extra calssifier to steet the gradient -> extra work

    training: 
    conditional/unconditional

    sampling:
    guidance scale /gamma

## performance metrics

    Frechet

    Inception score

    Precision & recall

## latent DDPM

    why latent space: 
    pixel level too expensice
    high dimensional data

    encoder/decoder

    stable diffusion
    DiT replace U-Net with transformers

## summary

VAE: one big step Diffusion model: many small steps

forward using fixed equation (no training needed)


##

    

1.  
    forward: scheduled fixed

    going back is more cahllenging

    How?

    stage the gaussian noiseing process

    1. each step add a little more gaussian (easy to go back)
    2. goback(denoise)

2.  
    why sde?

    step smaller and smaller -> continuous (stochastic)

    in forward process

3.  
    每一步denoise 

    X0<-X1<-X2

    shared parameter

4.  classifier
    
    classifier guided reconst..
    
    classifier看output并纠正

    classifier free how?





