#![allow(non_upper_case_globals, non_camel_case_types, non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

// functions below are translated from C++ to Rust, because they are defined inside the NvFlex.h header.

#[doc = "Generate a bit set for the particle phase, this is a helper method to simply combine the\ngroup id and bit flags into a single integer.\n\n@param[in] group The index of the group for this particle, should be an integer < 2^20\n@param[in] particleFlags A combination of the phase flags which should be a combination of eNvFlexPhaseSelfCollide, eNvFlexPhaseSelfCollideFilter, and eNvFlexPhaseFluid\n@param[in] shapeChannels A combination of eNvFlexPhaseShapeChannel* flags that control which shapes will be collided against, particles will only collide against shapes that share at least one set channel, see NvFlexMakeShapeFlagsWithChannels()"]
#[inline]
pub fn NvFlexMakePhaseWithChannels(
    group: std::ffi::c_int,
    particleFlags: std::ffi::c_int,
    shapeChannels: std::ffi::c_int,
) -> std::ffi::c_int {
    (group & eNvFlexPhaseGroupMask)
        | (particleFlags & eNvFlexPhaseFlagsMask)
        | (shapeChannels & eNvFlexPhaseShapeChannelMask)
}

#[doc = "Deprecated helper method to generates a phase with all shape channels set"]
#[inline]
pub fn NvFlexMakePhase(group: std::ffi::c_int, particleFlags: std::ffi::c_int) -> std::ffi::c_int {
    NvFlexMakePhaseWithChannels(group, particleFlags, eNvFlexPhaseShapeChannelMask)
}

#[doc = "Helper function to combine shape type, flags, and phase/shape collision channels into a 32bit value\n\n@param[in] type The type of the shape, see NvFlexCollisionShapeType\n@param[in] dynamic See eNvFlexShapeFlagDynamic\n@param[in] shapeChannels A combination of the eNvFlexPhaseShapeChannel* flags, collisions will only be processed between a particle and a shape if a channel is set on both the particle and shape, see NvFlexMakePhaseWithChannels()"]
#[inline]
pub fn NvFlexMakeShapeFlagsWithChannels(
    typ: NvFlexCollisionShapeType,
    dynamic: bool,
    shapeChannels: std::ffi::c_int,
) -> std::ffi::c_int {
    typ | (if dynamic { eNvFlexShapeFlagDynamic } else { 0 }) | shapeChannels
}

#[doc = "Deprecrated helper method that creates shape flags that by default have all collision channels enabled"]
#[inline]
pub fn NvFlexMakeShapeFlags(typ: NvFlexCollisionShapeType, dynamic: bool) -> std::ffi::c_int {
    NvFlexMakeShapeFlagsWithChannels(typ, dynamic, eNvFlexPhaseShapeChannelMask)
}
