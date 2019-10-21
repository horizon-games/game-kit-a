import { AABB, RayCastInput, RayCastOutput } from '../Collision/Collision';
import { DistanceProxy } from '../Collision/Distance';
import { EdgeShape } from '../Collision/Shapes/EdgeShape';
import { MassData, Shape } from '../Collision/Shapes/Shape';
import { Color } from '../Common/Draw';
import { Rot, Transform, Vec2, XY } from '../Common/Math';
import { Body } from '../Dynamics/Body';
import { Fixture } from '../Dynamics/Fixture';
import { TimeStep } from '../Dynamics/TimeStep';
import { World } from '../Dynamics/World';
import { ContactFilter, ContactListener, QueryCallback, RayCastCallback } from '../Dynamics/WorldCallbacks';
import { IParticleDef, ParticleFlag, ParticleHandle } from './Particle';
import { IParticleGroupDef, ParticleGroup, ParticleGroupFlag } from './ParticleGroup';
export declare class GrowableBuffer<T> {
    data: T[];
    count: number;
    capacity: number;
    allocator: () => T;
    constructor(allocator: () => T);
    Append(): number;
    Reserve(newCapacity: number): void;
    Grow(): void;
    Free(): void;
    Shorten(newEnd: number): void;
    Data(): T[];
    GetCount(): number;
    SetCount(newCount: number): void;
    GetCapacity(): number;
    RemoveIf(pred: (t: T) => boolean): void;
    Unique(pred: (a: T, b: T) => boolean): void;
}
export declare type ParticleIndex = number;
export declare class FixtureParticleQueryCallback extends QueryCallback {
    m_system: ParticleSystem;
    constructor(system: ParticleSystem);
    ShouldQueryParticleSystem(system: ParticleSystem): boolean;
    ReportFixture(fixture: Fixture): boolean;
    ReportParticle(system: ParticleSystem, index: number): boolean;
    ReportFixtureAndParticle(fixture: Fixture, childIndex: number, index: number): void;
}
export declare class ParticleContact {
    indexA: number;
    indexB: number;
    weight: number;
    normal: Vec2;
    flags: ParticleFlag;
    SetIndices(a: number, b: number): void;
    SetWeight(w: number): void;
    SetNormal(n: Vec2): void;
    SetFlags(f: ParticleFlag): void;
    GetIndexA(): number;
    GetIndexB(): number;
    GetWeight(): number;
    GetNormal(): Vec2;
    GetFlags(): ParticleFlag;
    IsEqual(rhs: ParticleContact): boolean;
    IsNotEqual(rhs: ParticleContact): boolean;
    ApproximatelyEqual(rhs: ParticleContact): boolean;
}
export declare class ParticleBodyContact {
    index: number;
    body: Body;
    fixture: Fixture;
    weight: number;
    normal: Vec2;
    mass: number;
}
export declare class ParticlePair {
    indexA: number;
    indexB: number;
    flags: ParticleFlag;
    strength: number;
    distance: number;
}
export declare class ParticleTriad {
    indexA: number;
    indexB: number;
    indexC: number;
    flags: ParticleFlag;
    strength: number;
    pa: Vec2;
    pb: Vec2;
    pc: Vec2;
    ka: number;
    kb: number;
    kc: number;
    s: number;
}
export declare class ParticleSystemDef {
    /**
     * Enable strict Particle/Body contact check.
     * See SetStrictContactCheck for details.
     */
    strictContactCheck: boolean;
    /**
     * Set the particle density.
     * See SetDensity for details.
     */
    density: number;
    /**
     * Change the particle gravity scale. Adjusts the effect of the
     * global gravity vector on particles. Default value is 1.0f.
     */
    gravityScale: number;
    /**
     * Particles behave as circles with this radius. In Box2D units.
     */
    radius: number;
    /**
     * Set the maximum number of particles.
     * By default, there is no maximum. The particle buffers can
     * continue to grow while World's block allocator still has
     * memory.
     * See SetMaxParticleCount for details.
     */
    maxCount: number;
    /**
     * Increases pressure in response to compression
     * Smaller values allow more compression
     */
    pressureStrength: number;
    /**
     * Reduces velocity along the collision normal
     * Smaller value reduces less
     */
    dampingStrength: number;
    /**
     * Restores shape of elastic particle groups
     * Larger values increase elastic particle velocity
     */
    elasticStrength: number;
    /**
     * Restores length of spring particle groups
     * Larger values increase spring particle velocity
     */
    springStrength: number;
    /**
     * Reduces relative velocity of viscous particles
     * Larger values slow down viscous particles more
     */
    viscousStrength: number;
    /**
     * Produces pressure on tensile particles
     * 0~0.2. Larger values increase the amount of surface tension.
     */
    surfaceTensionPressureStrength: number;
    /**
     * Smoothes outline of tensile particles
     * 0~0.2. Larger values result in rounder, smoother,
     * water-drop-like clusters of particles.
     */
    surfaceTensionNormalStrength: number;
    /**
     * Produces additional pressure on repulsive particles
     * Larger values repulse more
     * Negative values mean attraction. The range where particles
     * behave stably is about -0.2 to 2.0.
     */
    repulsiveStrength: number;
    /**
     * Produces repulsion between powder particles
     * Larger values repulse more
     */
    powderStrength: number;
    /**
     * Pushes particles out of solid particle group
     * Larger values repulse more
     */
    ejectionStrength: number;
    /**
     * Produces static pressure
     * Larger values increase the pressure on neighboring partilces
     * For a description of static pressure, see
     * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
     */
    staticPressureStrength: number;
    /**
     * Reduces instability in static pressure calculation
     * Larger values make stabilize static pressure with fewer
     * iterations
     */
    staticPressureRelaxation: number;
    /**
     * Computes static pressure more precisely
     * See SetStaticPressureIterations for details
     */
    staticPressureIterations: number;
    /**
     * Determines how fast colors are mixed
     * 1.0f ==> mixed immediately
     * 0.5f ==> mixed half way each simulation step (see
     * World::Step())
     */
    colorMixingStrength: number;
    /**
     * Whether to destroy particles by age when no more particles
     * can be created.  See #ParticleSystem::SetDestructionByAge()
     * for more information.
     */
    destroyByAge: boolean;
    /**
     * Granularity of particle lifetimes in seconds.  By default
     * this is set to (1.0f / 60.0f) seconds.  ParticleSystem uses
     * a 32-bit signed value to track particle lifetimes so the
     * maximum lifetime of a particle is (2^32 - 1) / (1.0f /
     * lifetimeGranularity) seconds. With the value set to 1/60 the
     * maximum lifetime or age of a particle is 2.27 years.
     */
    lifetimeGranularity: number;
    Copy(def: ParticleSystemDef): ParticleSystemDef;
    Clone(): ParticleSystemDef;
}
export declare class ParticleSystem {
    static readonly xTruncBits: number;
    static readonly yTruncBits: number;
    static readonly tagBits: number;
    static readonly yOffset: number;
    static readonly yShift: number;
    static readonly xShift: number;
    static readonly xScale: number;
    static readonly xOffset: number;
    static readonly yMask: number;
    static readonly xMask: number;
    static readonly DestroyParticlesInShape_s_aabb: AABB;
    static readonly CreateParticleGroup_s_transform: Transform;
    static readonly ComputeCollisionEnergy_s_v: Vec2;
    static readonly QueryShapeAABB_s_aabb: AABB;
    static readonly QueryPointAABB_s_aabb: AABB;
    static readonly RayCast_s_aabb: AABB;
    static readonly RayCast_s_p: Vec2;
    static readonly RayCast_s_v: Vec2;
    static readonly RayCast_s_n: Vec2;
    static readonly RayCast_s_point: Vec2;
    /**
     * All particle types that require creating pairs
     */
    static readonly k_pairFlags: number;
    /**
     * All particle types that require creating triads
     */
    static readonly k_triadFlags: ParticleFlag;
    /**
     * All particle types that do not produce dynamic pressure
     */
    static readonly k_noPressureFlags: number;
    /**
     * All particle types that apply extra damping force with bodies
     */
    static readonly k_extraDampingFlags: ParticleFlag;
    static readonly k_barrierWallFlags: number;
    static readonly CreateParticlesStrokeShapeForGroup_s_edge: EdgeShape;
    static readonly CreateParticlesStrokeShapeForGroup_s_d: Vec2;
    static readonly CreateParticlesStrokeShapeForGroup_s_p: Vec2;
    static readonly CreateParticlesFillShapeForGroup_s_aabb: AABB;
    static readonly CreateParticlesFillShapeForGroup_s_p: Vec2;
    static readonly AddContact_s_d: Vec2;
    static readonly UpdateBodyContacts_s_aabb: AABB;
    static readonly Solve_s_subStep: TimeStep;
    static readonly SolveCollision_s_aabb: AABB;
    static readonly SolveGravity_s_gravity: Vec2;
    static readonly SolveBarrier_s_aabb: AABB;
    static readonly SolveBarrier_s_va: Vec2;
    static readonly SolveBarrier_s_vb: Vec2;
    static readonly SolveBarrier_s_pba: Vec2;
    static readonly SolveBarrier_s_vba: Vec2;
    static readonly SolveBarrier_s_vc: Vec2;
    static readonly SolveBarrier_s_pca: Vec2;
    static readonly SolveBarrier_s_vca: Vec2;
    static readonly SolveBarrier_s_qba: Vec2;
    static readonly SolveBarrier_s_qca: Vec2;
    static readonly SolveBarrier_s_dv: Vec2;
    static readonly SolveBarrier_s_f: Vec2;
    static readonly SolvePressure_s_f: Vec2;
    static readonly SolveDamping_s_v: Vec2;
    static readonly SolveDamping_s_f: Vec2;
    static readonly SolveRigidDamping_s_t0: Vec2;
    static readonly SolveRigidDamping_s_t1: Vec2;
    static readonly SolveRigidDamping_s_p: Vec2;
    static readonly SolveRigidDamping_s_v: Vec2;
    static readonly SolveExtraDamping_s_v: Vec2;
    static readonly SolveExtraDamping_s_f: Vec2;
    static readonly SolveRigid_s_position: Vec2;
    static readonly SolveRigid_s_rotation: Rot;
    static readonly SolveRigid_s_transform: Transform;
    static readonly SolveRigid_s_velocityTransform: Transform;
    static readonly SolveElastic_s_pa: Vec2;
    static readonly SolveElastic_s_pb: Vec2;
    static readonly SolveElastic_s_pc: Vec2;
    static readonly SolveElastic_s_r: Rot;
    static readonly SolveElastic_s_t0: Vec2;
    static readonly SolveSpring_s_pa: Vec2;
    static readonly SolveSpring_s_pb: Vec2;
    static readonly SolveSpring_s_d: Vec2;
    static readonly SolveSpring_s_f: Vec2;
    static readonly SolveTensile_s_weightedNormal: Vec2;
    static readonly SolveTensile_s_s: Vec2;
    static readonly SolveTensile_s_f: Vec2;
    static readonly SolveViscous_s_v: Vec2;
    static readonly SolveViscous_s_f: Vec2;
    static readonly SolveRepulsive_s_f: Vec2;
    static readonly SolvePowder_s_f: Vec2;
    static readonly SolveSolid_s_f: Vec2;
    static computeTag(x: number, y: number): number;
    static computeRelativeTag(tag: number, x: number, y: number): number;
    static IsSignificantForce(force: XY): boolean;
    static ParticleCanBeConnected(flags: ParticleFlag, group: ParticleGroup | null): boolean;
    static ComparePairIndices(a: ParticlePair, b: ParticlePair): boolean;
    static MatchPairIndices(a: ParticlePair, b: ParticlePair): boolean;
    static CompareTriadIndices(a: ParticleTriad, b: ParticleTriad): boolean;
    static MatchTriadIndices(a: ParticleTriad, b: ParticleTriad): boolean;
    static InitializeParticleLists(group: ParticleGroup, nodeBuffer: ParticleSystemParticleListNode[]): void;
    static MergeParticleLists(listA: ParticleSystemParticleListNode, listB: ParticleSystemParticleListNode): void;
    static FindLongestParticleList(group: ParticleGroup, nodeBuffer: ParticleSystemParticleListNode[]): ParticleSystemParticleListNode;
    static MergeParticleListAndNode(list: ParticleSystemParticleListNode, node: ParticleSystemParticleListNode): void;
    static ParticleContactIsZombie(contact: ParticleContact): boolean;
    static BodyContactCompare(lhs: ParticleBodyContact, rhs: ParticleBodyContact): boolean;
    private static UpdatePairsAndTriads_s_dab;
    private static UpdatePairsAndTriads_s_dbc;
    private static UpdatePairsAndTriads_s_dca;
    private static RemoveSpuriousBodyContacts_s_n;
    private static RemoveSpuriousBodyContacts_s_pos;
    private static RemoveSpuriousBodyContacts_s_normal;
    m_paused: boolean;
    m_timestamp: number;
    m_allParticleFlags: ParticleFlag;
    m_needsUpdateAllParticleFlags: boolean;
    m_allGroupFlags: ParticleGroupFlag;
    m_needsUpdateAllGroupFlags: boolean;
    m_hasForce: boolean;
    m_iterationIndex: number;
    m_inverseDensity: number;
    m_particleDiameter: number;
    m_inverseDiameter: number;
    m_squaredDiameter: number;
    m_count: number;
    m_internalAllocatedCapacity: number;
    /**
     * Allocator for ParticleHandle instances.
     */
    /**
     * Maps particle indicies to handles.
     */
    m_handleIndexBuffer: ParticleSystemUserOverridableBuffer<ParticleHandle | null>;
    m_flagsBuffer: ParticleSystemUserOverridableBuffer<ParticleFlag>;
    m_positionBuffer: ParticleSystemUserOverridableBuffer<Vec2>;
    m_velocityBuffer: ParticleSystemUserOverridableBuffer<Vec2>;
    m_forceBuffer: Vec2[];
    /**
     * this.m_weightBuffer is populated in ComputeWeight and used in
     * ComputeDepth(), SolveStaticPressure() and SolvePressure().
     */
    m_weightBuffer: number[];
    /**
     * When any particles have the flag staticPressureParticle,
     * this.m_staticPressureBuffer is first allocated and used in
     * SolveStaticPressure() and SolvePressure().  It will be
     * reallocated on subsequent CreateParticle() calls.
     */
    m_staticPressureBuffer: number[];
    /**
     * this.m_accumulationBuffer is used in many functions as a temporary
     * buffer for scalar values.
     */
    m_accumulationBuffer: number[];
    /**
     * When any particles have the flag tensileParticle,
     * this.m_accumulation2Buffer is first allocated and used in
     * SolveTensile() as a temporary buffer for vector values.  It
     * will be reallocated on subsequent CreateParticle() calls.
     */
    m_accumulation2Buffer: Vec2[];
    /**
     * When any particle groups have the flag solidParticleGroup,
     * this.m_depthBuffer is first allocated and populated in
     * ComputeDepth() and used in SolveSolid(). It will be
     * reallocated on subsequent CreateParticle() calls.
     */
    m_depthBuffer: number[];
    m_colorBuffer: ParticleSystemUserOverridableBuffer<Color>;
    m_groupBuffer: Array<ParticleGroup | null>;
    m_userDataBuffer: ParticleSystemUserOverridableBuffer<any>;
    /**
     * Stuck particle detection parameters and record keeping
     */
    m_stuckThreshold: number;
    m_lastBodyContactStepBuffer: ParticleSystemUserOverridableBuffer<number>;
    m_bodyContactCountBuffer: ParticleSystemUserOverridableBuffer<number>;
    m_consecutiveContactStepsBuffer: ParticleSystemUserOverridableBuffer<number>;
    m_stuckParticleBuffer: GrowableBuffer<number>;
    m_proxyBuffer: GrowableBuffer<ParticleSystemProxy>;
    m_contactBuffer: GrowableBuffer<ParticleContact>;
    m_bodyContactBuffer: GrowableBuffer<ParticleBodyContact>;
    m_pairBuffer: GrowableBuffer<ParticlePair>;
    m_triadBuffer: GrowableBuffer<ParticleTriad>;
    /**
     * Time each particle should be destroyed relative to the last
     * time this.m_timeElapsed was initialized.  Each unit of time
     * corresponds to ParticleSystemDef::lifetimeGranularity
     * seconds.
     */
    m_expirationTimeBuffer: ParticleSystemUserOverridableBuffer<number>;
    /**
     * List of particle indices sorted by expiration time.
     */
    m_indexByExpirationTimeBuffer: ParticleSystemUserOverridableBuffer<number>;
    /**
     * Time elapsed in 32:32 fixed point.  Each non-fractional unit
     * of time corresponds to
     * ParticleSystemDef::lifetimeGranularity seconds.
     */
    m_timeElapsed: number;
    /**
     * Whether the expiration time buffer has been modified and
     * needs to be resorted.
     */
    m_expirationTimeBufferRequiresSorting: boolean;
    m_groupCount: number;
    m_groupList: ParticleGroup | null;
    m_def: ParticleSystemDef;
    m_world: World;
    m_prev: ParticleSystem | null;
    m_next: ParticleSystem | null;
    constructor(def: ParticleSystemDef, world: World);
    Drop(): void;
    /**
     * Create a particle whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * A simulation step must occur before it's possible to interact
     * with a newly created particle.  For example,
     * DestroyParticleInShape() will not destroy a particle until
     * World::Step() has been called.
     *
     * warning: This function is locked during callbacks.
     */
    CreateParticle(def: IParticleDef): number;
    /**
     * Retrieve a handle to the particle at the specified index.
     *
     * Please see #ParticleHandle for why you might want a handle.
     */
    GetParticleHandleFromIndex(index: number): ParticleHandle;
    /**
     * Destroy a particle.
     *
     * The particle is removed after the next simulation step (see
     * World::Step()).
     *
     * @param index Index of the particle to destroy.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    DestroyParticle(index: number, callDestructionListener?: boolean): void;
    /**
     * Destroy the Nth oldest particle in the system.
     *
     * The particle is removed after the next World::Step().
     *
     * @param index Index of the Nth oldest particle to
     *      destroy, 0 will destroy the oldest particle in the
     *      system, 1 will destroy the next oldest particle etc.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    DestroyOldestParticle(index: number, callDestructionListener?: boolean): void;
    /**
     * Destroy particles inside a shape.
     *
     * warning: This function is locked during callbacks.
     *
     * In addition, this function immediately destroys particles in
     * the shape in constrast to DestroyParticle() which defers the
     * destruction until the next simulation step.
     *
     * @return Number of particles destroyed.
     * @param shape Shape which encloses particles
     *      that should be destroyed.
     * @param xf Transform applied to the shape.
     * @param callDestructionListener Whether to call the
     *      world DestructionListener for each particle
     *      destroyed.
     */
    DestroyParticlesInShape(shape: Shape, xf: Transform, callDestructionListener?: boolean): number;
    /**
     * Create a particle group whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * warning: This function is locked during callbacks.
     */
    CreateParticleGroup(groupDef: IParticleGroupDef): ParticleGroup;
    /**
     * Join two particle groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param groupA the first group. Expands to encompass the second group.
     * @param groupB the second group. It is destroyed.
     */
    JoinParticleGroups(groupA: ParticleGroup, groupB: ParticleGroup): void;
    /**
     * Split particle group into multiple disconnected groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param group the group to be split.
     */
    SplitParticleGroup(group: ParticleGroup): void;
    /**
     * Get the world particle group list. With the returned group,
     * use ParticleGroup::GetNext to get the next group in the
     * world list.
     *
     * A null group indicates the end of the list.
     *
     * @return the head of the world particle group list.
     */
    GetParticleGroupList(): ParticleGroup | null;
    /**
     * Get the number of particle groups.
     */
    GetParticleGroupCount(): number;
    /**
     * Get the number of particles.
     */
    GetParticleCount(): number;
    /**
     * Get the maximum number of particles.
     */
    GetMaxParticleCount(): number;
    /**
     * Set the maximum number of particles.
     *
     * A value of 0 means there is no maximum. The particle buffers
     * can continue to grow while World's block allocator still
     * has memory.
     *
     * Note: If you try to CreateParticle() with more than this
     * count, invalidParticleIndex is returned unless
     * SetDestructionByAge() is used to enable the destruction of
     * the oldest particles in the system.
     */
    SetMaxParticleCount(count: number): void;
    /**
     * Get all existing particle flags.
     */
    GetAllParticleFlags(): ParticleFlag;
    /**
     * Get all existing particle group flags.
     */
    GetAllGroupFlags(): ParticleGroupFlag;
    /**
     * Pause or unpause the particle system. When paused,
     * World::Step() skips over this particle system. All
     * ParticleSystem function calls still work.
     *
     * @param paused paused is true to pause, false to un-pause.
     */
    SetPaused(paused: boolean): void;
    /**
     * Initially, true, then, the last value passed into
     * SetPaused().
     *
     * @return true if the particle system is being updated in World::Step().
     */
    GetPaused(): boolean;
    /**
     * Change the particle density.
     *
     * Particle density affects the mass of the particles, which in
     * turn affects how the particles interact with Bodies. Note
     * that the density does not affect how the particles interact
     * with each other.
     */
    SetDensity(density: number): void;
    /**
     * Get the particle density.
     */
    GetDensity(): number;
    /**
     * Change the particle gravity scale. Adjusts the effect of the
     * global gravity vector on particles.
     */
    SetGravityScale(gravityScale: number): void;
    /**
     * Get the particle gravity scale.
     */
    GetGravityScale(): number;
    /**
     * Damping is used to reduce the velocity of particles. The
     * damping parameter can be larger than 1.0f but the damping
     * effect becomes sensitive to the time step when the damping
     * parameter is large.
     */
    SetDamping(damping: number): void;
    /**
     * Get damping for particles
     */
    GetDamping(): number;
    /**
     * Change the number of iterations when calculating the static
     * pressure of particles. By default, 8 iterations. You can
     * reduce the number of iterations down to 1 in some situations,
     * but this may cause instabilities when many particles come
     * together. If you see particles popping away from each other
     * like popcorn, you may have to increase the number of
     * iterations.
     *
     * For a description of static pressure, see
     * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
     */
    SetStaticPressureIterations(iterations: number): void;
    /**
     * Get the number of iterations for static pressure of
     * particles.
     */
    GetStaticPressureIterations(): number;
    /**
     * Change the particle radius.
     *
     * You should set this only once, on world start.
     * If you change the radius during execution, existing particles
     * may explode, shrink, or behave unexpectedly.
     */
    SetRadius(radius: number): void;
    /**
     * Get the particle radius.
     */
    GetRadius(): number;
    /**
     * Get the position of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle positions array.
     */
    GetPositionBuffer(): Vec2[];
    /**
     * Get the velocity of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle velocities array.
     */
    GetVelocityBuffer(): Vec2[];
    /**
     * Get the color of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle colors array.
     */
    GetColorBuffer(): Color[];
    /**
     * Get the particle-group of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle group array.
     */
    GetGroupBuffer(): Array<ParticleGroup | null>;
    /**
     * Get the weight of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle positions array.
     */
    GetWeightBuffer(): number[];
    /**
     * Get the user-specified data of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle user-data array.
     */
    GetUserDataBuffer<T>(): T[];
    /**
     * Get the flags for each particle. See the ParticleFlag enum.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle-flags array.
     */
    GetFlagsBuffer(): ParticleFlag[];
    /**
     * Set flags for a particle. See the ParticleFlag enum.
     */
    SetParticleFlags(index: number, newFlags: ParticleFlag): void;
    /**
     * Get flags for a particle. See the ParticleFlag enum.
     */
    GetParticleFlags(index: number): ParticleFlag;
    /**
     * Set an external buffer for particle data.
     *
     * Normally, the World's block allocator is used for particle
     * data. However, sometimes you may have an OpenGL or Java
     * buffer for particle data. To avoid data duplication, you may
     * supply this external buffer.
     *
     * Note that, when World's block allocator is used, the
     * particle data buffers can grow as required. However, when
     * external buffers are used, the maximum number of particles is
     * clamped to the size of the smallest external buffer.
     *
     * @param buffer a pointer to a block of memory.
     * @param capacity the number of values in the block.
     */
    SetFlagsBuffer(buffer: ParticleFlag[], capacity: number): void;
    SetPositionBuffer(buffer: Vec2[], capacity: number): void;
    SetVelocityBuffer(buffer: Vec2[], capacity: number): void;
    SetColorBuffer(buffer: Color[], capacity: number): void;
    SetUserDataBuffer<T>(buffer: T[], capacity: number): void;
    /**
     * Get contacts between particles
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    GetContacts(): ParticleContact[];
    GetContactCount(): number;
    /**
     * Get contacts between particles and bodies
     *
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    GetBodyContacts(): ParticleBodyContact[];
    GetBodyContactCount(): number;
    /**
     * Get array of particle pairs. The particles in a pair:
     *   (1) are contacting,
     *   (2) are in the same particle group,
     *   (3) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (4) have at least one particle that is a spring or barrier
     *       particle (i.e. one of the types in k_pairFlags),
     *   (5) have at least one particle that returns true for
     *       ConnectionFilter::IsNecessary,
     *   (6) are not zombie particles.
     *
     * Essentially, this is an array of spring or barrier particles
     * that are interacting. The array is sorted by ParticlePair's
     * indexA, and then indexB. There are no duplicate entries.
     */
    GetPairs(): ParticlePair[];
    GetPairCount(): number;
    /**
     * Get array of particle triads. The particles in a triad:
     *   (1) are in the same particle group,
     *   (2) are in a Voronoi triangle together,
     *   (3) are within maxTriadDistance particle diameters of each
     *       other,
     *   (4) return true for ConnectionFilter::ShouldCreateTriad
     *   (5) have at least one particle of type elastic (i.e. one of the
     *       types in k_triadFlags),
     *   (6) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (7) are not zombie particles.
     *
     * Essentially, this is an array of elastic particles that are
     * interacting. The array is sorted by ParticleTriad's indexA,
     * then indexB, then indexC. There are no duplicate entries.
     */
    GetTriads(): ParticleTriad[];
    GetTriadCount(): number;
    /**
     * Set an optional threshold for the maximum number of
     * consecutive particle iterations that a particle may contact
     * multiple bodies before it is considered a candidate for being
     * "stuck". Setting to zero or less disables.
     */
    SetStuckThreshold(steps: number): void;
    /**
     * Get potentially stuck particles from the last step; the user
     * must decide if they are stuck or not, and if so, delete or
     * move them
     */
    GetStuckCandidates(): number[];
    /**
     * Get the number of stuck particle candidates from the last
     * step.
     */
    GetStuckCandidateCount(): number;
    /**
     * Compute the kinetic energy that can be lost by damping force
     */
    ComputeCollisionEnergy(): number;
    /**
     * Set strict Particle/Body contact check.
     *
     * This is an option that will help ensure correct behavior if
     * there are corners in the world model where Particle/Body
     * contact is ambiguous. This option scales at n*log(n) of the
     * number of Particle/Body contacts, so it is best to only
     * enable if it is necessary for your geometry. Enable if you
     * see strange particle behavior around Body intersections.
     */
    SetStrictContactCheck(enabled: boolean): void;
    /**
     * Get the status of the strict contact check.
     */
    GetStrictContactCheck(): boolean;
    /**
     * Set the lifetime (in seconds) of a particle relative to the
     * current time.  A lifetime of less than or equal to 0.0f
     * results in the particle living forever until it's manually
     * destroyed by the application.
     */
    SetParticleLifetime(index: number, lifetime: number): void;
    /**
     * Get the lifetime (in seconds) of a particle relative to the
     * current time.  A value > 0.0f is returned if the particle is
     * scheduled to be destroyed in the future, values <= 0.0f
     * indicate the particle has an infinite lifetime.
     */
    GetParticleLifetime(index: number): number;
    /**
     * Enable / disable destruction of particles in CreateParticle()
     * when no more particles can be created due to a prior call to
     * SetMaxParticleCount().  When this is enabled, the oldest
     * particle is destroyed in CreateParticle() favoring the
     * destruction of particles with a finite lifetime over
     * particles with infinite lifetimes. This feature is enabled by
     * default when particle lifetimes are tracked.  Explicitly
     * enabling this feature using this function enables particle
     * lifetime tracking.
     */
    SetDestructionByAge(enable: boolean): void;
    /**
     * Get whether the oldest particle will be destroyed in
     * CreateParticle() when the maximum number of particles are
     * present in the system.
     */
    GetDestructionByAge(): boolean;
    /**
     * Get the array of particle expiration times indexed by
     * particle index.
     *
     * GetParticleCount() items are in the returned array.
     */
    GetExpirationTimeBuffer(): number[];
    /**
     * Convert a expiration time value in returned by
     * GetExpirationTimeBuffer() to a time in seconds relative to
     * the current simulation time.
     */
    ExpirationTimeToLifetime(expirationTime: number): number;
    /**
     * Get the array of particle indices ordered by reverse
     * lifetime. The oldest particle indexes are at the end of the
     * array with the newest at the start.  Particles with infinite
     * lifetimes (i.e expiration times less than or equal to 0) are
     * placed at the start of the array.
     * ExpirationTimeToLifetime(GetExpirationTimeBuffer()[index]) is
     * equivalent to GetParticleLifetime(index).
     *
     * GetParticleCount() items are in the returned array.
     */
    GetIndexByExpirationTimeBuffer(): number[];
    /**
     * Apply an impulse to one particle. This immediately modifies
     * the velocity. Similar to Body::ApplyLinearImpulse.
     *
     * @param index the particle that will be modified.
     * @param impulse impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    ParticleApplyLinearImpulse(index: number, impulse: XY): void;
    /**
     * Apply an impulse to all particles between 'firstIndex' and
     * 'lastIndex'. This immediately modifies the velocity. Note
     * that the impulse is applied to the total mass of all
     * particles. So, calling ParticleApplyLinearImpulse(0, impulse)
     * and ParticleApplyLinearImpulse(1, impulse) will impart twice
     * as much velocity as calling just ApplyLinearImpulse(0, 1,
     * impulse).
     *
     * @param firstIndex the first particle to be modified.
     * @param lastIndex the last particle to be modified.
     * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    ApplyLinearImpulse(firstIndex: number, lastIndex: number, impulse: XY): void;
    /**
     * Apply a force to the center of a particle.
     *
     * @param index the particle that will be modified.
     * @param force the world force vector, usually in Newtons (N).
     */
    ParticleApplyForce(index: number, force: XY): void;
    /**
     * Distribute a force across several particles. The particles
     * must not be wall particles. Note that the force is
     * distributed across all the particles, so calling this
     * function for indices 0..N is not the same as calling
     * ParticleApplyForce(i, force) for i in 0..N.
     *
     * @param firstIndex the first particle to be modified.
     * @param lastIndex the last particle to be modified.
     * @param force the world force vector, usually in Newtons (N).
     */
    ApplyForce(firstIndex: number, lastIndex: number, force: XY): void;
    /**
     * Get the next particle-system in the world's particle-system
     * list.
     */
    GetNext(): ParticleSystem | null;
    /**
     * Query the particle system for all particles that potentially
     * overlap the provided AABB.
     * QueryCallback::ShouldQueryParticleSystem is ignored.
     *
     * @param callback a user implemented callback class.
     * @param aabb the query box.
     */
    QueryAABB(callback: QueryCallback, aabb: AABB): void;
    /**
     * Query the particle system for all particles that potentially
     * overlap the provided shape's AABB. Calls QueryAABB
     * internally. QueryCallback::ShouldQueryParticleSystem is
     * ignored.
     *
     * @param callback a user implemented callback class.
     * @param shape the query shape
     * @param xf the transform of the AABB
     * @param childIndex
     */
    QueryShapeAABB(callback: QueryCallback, shape: Shape, xf: Transform, childIndex?: number): void;
    QueryPointAABB(callback: QueryCallback, point: Vec2, slop?: number): void;
    /**
     * Ray-cast the particle system for all particles in the path of
     * the ray. Your callback controls whether you get the closest
     * point, any point, or n-points. The ray-cast ignores particles
     * that contain the starting point.
     * RayCastCallback::ShouldQueryParticleSystem is ignored.
     *
     * @param callback a user implemented callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    RayCast(callback: RayCastCallback, point1: Vec2, point2: Vec2): void;
    /**
     * Compute the axis-aligned bounding box for all particles
     * contained within this particle system.
     * @param aabb Returns the axis-aligned bounding box of the system.
     */
    ComputeAABB(aabb: AABB): void;
    FreeBuffer<T>(b: T[] | null, capacity: number): void;
    FreeUserOverridableBuffer<T>(b: ParticleSystemUserOverridableBuffer<T>): void;
    /**
     * Reallocate a buffer
     */
    ReallocateBuffer3<T>(oldBuffer: T[] | null, oldCapacity: number, newCapacity: number): T[];
    /**
     * Reallocate a buffer
     */
    ReallocateBuffer5<T>(buffer: T[] | null, userSuppliedCapacity: number, oldCapacity: number, newCapacity: number, deferred: boolean): T[];
    /**
     * Reallocate a buffer
     */
    ReallocateBuffer4<T>(buffer: ParticleSystemUserOverridableBuffer<any>, oldCapacity: number, newCapacity: number, deferred: boolean): T[] | null;
    RequestBuffer<T>(buffer: T[] | null): T[];
    /**
     * Reallocate the handle / index map and schedule the allocation
     * of a new pool for handle allocation.
     */
    ReallocateHandleBuffers(newCapacity: number): void;
    ReallocateInternalAllocatedBuffers(capacity: number): void;
    CreateParticleForGroup(groupDef: IParticleGroupDef, xf: Transform, p: XY): void;
    CreateParticlesStrokeShapeForGroup(shape: Shape, groupDef: IParticleGroupDef, xf: Transform): void;
    CreateParticlesFillShapeForGroup(shape: Shape, groupDef: IParticleGroupDef, xf: Transform): void;
    CreateParticlesWithShapeForGroup(shape: Shape, groupDef: IParticleGroupDef, xf: Transform): void;
    CreateParticlesWithShapesForGroup(shapes: Shape[], shapeCount: number, groupDef: IParticleGroupDef, xf: Transform): void;
    CloneParticle(oldIndex: number, group: ParticleGroup): number;
    DestroyParticlesInGroup(group: ParticleGroup, callDestructionListener?: boolean): void;
    DestroyParticleGroup(group: ParticleGroup): void;
    UpdatePairsAndTriads(firstIndex: number, lastIndex: number, filter: ParticleSystemConnectionFilter): void;
    UpdatePairsAndTriadsWithReactiveParticles(): void;
    MergeParticleListsInContact(group: ParticleGroup, nodeBuffer: ParticleSystemParticleListNode[]): void;
    MergeZombieParticleListNodes(group: ParticleGroup, nodeBuffer: ParticleSystemParticleListNode[], survivingList: ParticleSystemParticleListNode): void;
    CreateParticleGroupsFromParticleList(group: ParticleGroup, nodeBuffer: ParticleSystemParticleListNode[], survivingList: ParticleSystemParticleListNode): void;
    UpdatePairsAndTriadsWithParticleList(group: ParticleGroup, nodeBuffer: ParticleSystemParticleListNode[]): void;
    ComputeDepth(): void;
    GetInsideBoundsEnumerator(aabb: Readonly<AABB>): ParticleSystemInsideBoundsEnumerator;
    UpdateAllParticleFlags(): void;
    UpdateAllGroupFlags(): void;
    AddContact(a: number, b: number, contacts: GrowableBuffer<ParticleContact>): void;
    FindContacts_Reference(contacts: GrowableBuffer<ParticleContact>): void;
    FindContacts(contacts: GrowableBuffer<ParticleContact>): void;
    UpdateProxies_Reference(proxies: GrowableBuffer<ParticleSystemProxy>): void;
    UpdateProxies(proxies: GrowableBuffer<ParticleSystemProxy>): void;
    SortProxies(proxies: GrowableBuffer<ParticleSystemProxy>): void;
    FilterContacts(contacts: GrowableBuffer<ParticleContact>): void;
    NotifyContactListenerPreContact(particlePairs: ParticlePairSet): void;
    NotifyContactListenerPostContact(particlePairs: ParticlePairSet): void;
    UpdateContacts(exceptZombie: boolean): void;
    NotifyBodyContactListenerPreContact(fixtureSet: ParticleSystemFixtureParticleSet): void;
    NotifyBodyContactListenerPostContact(fixtureSet: ParticleSystemFixtureParticleSet): void;
    UpdateBodyContacts(): void;
    Solve(step: TimeStep): void;
    SolveCollision(step: TimeStep): void;
    LimitVelocity(step: TimeStep): void;
    SolveGravity(step: TimeStep): void;
    SolveBarrier(step: TimeStep): void;
    SolveStaticPressure(step: TimeStep): void;
    ComputeWeight(): void;
    SolvePressure(step: TimeStep): void;
    SolveDamping(step: TimeStep): void;
    SolveRigidDamping(): void;
    SolveExtraDamping(): void;
    SolveWall(): void;
    SolveRigid(step: TimeStep): void;
    SolveElastic(step: TimeStep): void;
    SolveSpring(step: TimeStep): void;
    SolveTensile(step: TimeStep): void;
    SolveViscous(): void;
    SolveRepulsive(step: TimeStep): void;
    SolvePowder(step: TimeStep): void;
    SolveSolid(step: TimeStep): void;
    SolveForce(step: TimeStep): void;
    SolveColorMixing(): void;
    SolveZombie(): void;
    /**
     * Destroy all particles which have outlived their lifetimes set
     * by SetParticleLifetime().
     */
    SolveLifetimes(step: TimeStep): void;
    RotateBuffer(start: number, mid: number, end: number): void;
    GetCriticalVelocity(step: TimeStep): number;
    GetCriticalVelocitySquared(step: TimeStep): number;
    GetCriticalPressure(step: TimeStep): number;
    GetParticleStride(): number;
    GetParticleMass(): number;
    GetParticleInvMass(): number;
    /**
     * Get the world's contact filter if any particles with the
     * contactFilterParticle flag are present in the system.
     */
    GetFixtureContactFilter(): ContactFilter | null;
    /**
     * Get the world's contact filter if any particles with the
     * particleContactFilterParticle flag are present in the
     * system.
     */
    GetParticleContactFilter(): ContactFilter | null;
    /**
     * Get the world's contact listener if any particles with the
     * fixtureContactListenerParticle flag are present in the
     * system.
     */
    GetFixtureContactListener(): ContactListener | null;
    /**
     * Get the world's contact listener if any particles with the
     * particleContactListenerParticle flag are present in the
     * system.
     */
    GetParticleContactListener(): ContactListener | null;
    SetUserOverridableBuffer<T>(buffer: ParticleSystemUserOverridableBuffer<any>, newData: T[], newCapacity: number): void;
    SetGroupFlags(group: ParticleGroup, newFlags: ParticleGroupFlag): void;
    RemoveSpuriousBodyContacts(): void;
    DetectStuckParticle(particle: number): void;
    /**
     * Determine whether a particle index is valid.
     */
    ValidateParticleIndex(index: number): boolean;
    /**
     * Get the time elapsed in
     * ParticleSystemDef::lifetimeGranularity.
     */
    GetQuantizedTimeElapsed(): number;
    /**
     * Convert a lifetime in seconds to an expiration time.
     */
    LifetimeToExpirationTime(lifetime: number): number;
    ForceCanBeApplied(flags: ParticleFlag): boolean;
    PrepareForceBuffer(): void;
    IsRigidGroup(group: ParticleGroup | null): boolean;
    GetLinearVelocity(group: ParticleGroup | null, particleIndex: number, point: Vec2, out: Vec2): Vec2;
    InitDampingParameter(invMass: number[], invInertia: number[], tangentDistance: number[], mass: number, inertia: number, center: Vec2, point: Vec2, normal: Vec2): void;
    InitDampingParameterWithRigidGroupOrParticle(invMass: number[], invInertia: number[], tangentDistance: number[], isRigidGroup: boolean, group: ParticleGroup | null, particleIndex: number, point: Vec2, normal: Vec2): void;
    ComputeDampingImpulse(invMassA: number, invInertiaA: number, tangentDistanceA: number, invMassB: number, invInertiaB: number, tangentDistanceB: number, normalVelocity: number): number;
    ApplyDamping(invMass: number, invInertia: number, tangentDistance: number, isRigidGroup: boolean, group: ParticleGroup | null, particleIndex: number, impulse: number, normal: Vec2): void;
}
export declare class ParticleSystemUserOverridableBuffer<T> {
    data: T[] | null;
    userSuppliedCapacity: number;
}
export declare class ParticleSystemProxy {
    static CompareProxyProxy(a: ParticleSystemProxy, b: ParticleSystemProxy): boolean;
    static CompareTagProxy(a: number, b: ParticleSystemProxy): boolean;
    static CompareProxyTag(a: ParticleSystemProxy, b: number): boolean;
    index: number;
    tag: number;
}
export declare class ParticleSystemInsideBoundsEnumerator {
    m_system: ParticleSystem;
    m_xLower: number;
    m_xUpper: number;
    m_yLower: number;
    m_yUpper: number;
    m_first: number;
    m_last: number;
    /**
     * InsideBoundsEnumerator enumerates all particles inside the
     * given bounds.
     *
     * Construct an enumerator with bounds of tags and a range of
     * proxies.
     */
    constructor(system: ParticleSystem, lower: number, upper: number, first: number, last: number);
    /**
     * Get index of the next particle. Returns
     * invalidParticleIndex if there are no more particles.
     */
    GetNext(): number;
}
export declare class ParticleSystemParticleListNode {
    /**
     * The head of the list.
     */
    list: ParticleSystemParticleListNode;
    /**
     * The next node in the list.
     */
    next: ParticleSystemParticleListNode | null;
    /**
     * Number of entries in the list. Valid only for the node at the
     * head of the list.
     */
    count: number;
    /**
     * Particle index.
     */
    index: number;
}
/**
 * @constructor
 */
export declare class ParticleSystemFixedSetAllocator<T> {
    Allocate(itemSize: number, count: number): number;
    Clear(): void;
    GetCount(): number;
    Invalidate(itemIndex: number): void;
    GetValidBuffer(): boolean[];
    GetBuffer(): T[];
    SetCount(count: number): void;
}
export declare class ParticleSystemFixtureParticle {
    first: Fixture;
    second: number;
    constructor(fixture: Fixture, particle: number);
}
export declare class ParticleSystemFixtureParticleSet extends ParticleSystemFixedSetAllocator<ParticleSystemFixtureParticle> {
    Initialize(bodyContactBuffer: GrowableBuffer<ParticleBodyContact>, flagsBuffer: ParticleSystemUserOverridableBuffer<ParticleFlag>): void;
    Find(pair: ParticleSystemFixtureParticle): number;
}
export declare class ParticleSystemParticlePair {
    first: number;
    second: number;
    constructor(particleA: number, particleB: number);
}
export declare class ParticlePairSet extends ParticleSystemFixedSetAllocator<ParticleSystemParticlePair> {
    Initialize(contactBuffer: GrowableBuffer<ParticleContact>, flagsBuffer: ParticleSystemUserOverridableBuffer<ParticleFlag>): void;
    Find(pair: ParticleSystemParticlePair): number;
}
export declare class ParticleSystemConnectionFilter {
    /**
     * Is the particle necessary for connection?
     * A pair or a triad should contain at least one 'necessary'
     * particle.
     */
    IsNecessary(index: number): boolean;
    /**
     * An additional condition for creating a pair.
     */
    ShouldCreatePair(a: number, b: number): boolean;
    /**
     * An additional condition for creating a triad.
     */
    ShouldCreateTriad(a: number, b: number, c: number): boolean;
}
export declare class ParticleSystemDestroyParticlesInShapeCallback extends QueryCallback {
    m_system: ParticleSystem;
    m_shape: Shape;
    m_xf: Transform;
    m_callDestructionListener: boolean;
    m_destroyed: number;
    constructor(system: ParticleSystem, shape: Shape, xf: Transform, callDestructionListener: boolean);
    ReportFixture(fixture: Fixture): boolean;
    ReportParticle(particleSystem: ParticleSystem, index: number): boolean;
    Destroyed(): number;
}
export declare class ParticleSystemJoinParticleGroupsFilter extends ParticleSystemConnectionFilter {
    m_threshold: number;
    constructor(threshold: number);
    /**
     * An additional condition for creating a pair.
     */
    ShouldCreatePair(a: number, b: number): boolean;
    /**
     * An additional condition for creating a triad.
     */
    ShouldCreateTriad(a: number, b: number, c: number): boolean;
}
export declare class ParticleSystemCompositeShape extends Shape {
    m_shapes: Shape[];
    m_shapeCount: number;
    constructor(shapes: Shape[], shapeCount?: number);
    Clone(): Shape;
    GetChildCount(): number;
    /**
     * @see Shape::TestPoint
     */
    TestPoint(xf: Transform, p: Vec2): boolean;
    /**
     * @see Shape::ComputeDistance
     */
    ComputeDistance(xf: Transform, p: Vec2, normal: Vec2, childIndex: number): number;
    /**
     * Implement Shape.
     */
    RayCast(output: RayCastOutput, input: RayCastInput, xf: Transform, childIndex: number): boolean;
    /**
     * @see Shape::ComputeAABB
     */
    ComputeAABB(aabb: AABB, xf: Transform, childIndex: number): void;
    /**
     * @see Shape::ComputeMass
     */
    ComputeMass(massData: MassData, density: number): void;
    SetupDistanceProxy(proxy: DistanceProxy, index: number): void;
    ComputeSubmergedArea(normal: Vec2, offset: number, xf: Transform, c: Vec2): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
export declare class ParticleSystemReactiveFilter extends ParticleSystemConnectionFilter {
    m_flagsBuffer: ParticleSystemUserOverridableBuffer<ParticleFlag>;
    constructor(flagsBuffer: ParticleSystemUserOverridableBuffer<ParticleFlag>);
    IsNecessary(index: number): boolean;
}
export declare class ParticleSystemUpdateBodyContactsCallback extends FixtureParticleQueryCallback {
    static readonly ReportFixtureAndParticle_s_n: Vec2;
    static readonly ReportFixtureAndParticle_s_rp: Vec2;
    m_contactFilter: ContactFilter | null;
    constructor(system: ParticleSystem, contactFilter: ContactFilter | null);
    ShouldCollideFixtureParticle(fixture: Fixture, particleSystem: ParticleSystem, particleIndex: number): boolean;
    ReportFixtureAndParticle(fixture: Fixture, childIndex: number, a: number): void;
}
export declare class ParticleSystemSolveCollisionCallback extends FixtureParticleQueryCallback {
    static readonly ReportFixtureAndParticle_s_p1: Vec2;
    static readonly ReportFixtureAndParticle_s_output: RayCastOutput;
    static readonly ReportFixtureAndParticle_s_input: RayCastInput;
    static readonly ReportFixtureAndParticle_s_p: Vec2;
    static readonly ReportFixtureAndParticle_s_v: Vec2;
    static readonly ReportFixtureAndParticle_s_f: Vec2;
    m_step: TimeStep;
    constructor(system: ParticleSystem, step: TimeStep);
    ReportFixtureAndParticle(fixture: Fixture, childIndex: number, a: number): void;
    ReportParticle(system: ParticleSystem, index: number): boolean;
}
