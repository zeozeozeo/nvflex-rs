use crate::{rstr, InitDesc, SolverDesc, SolverParams};
use nvflex_sys::*;

/// Flex error return codes
#[derive(Debug, Copy, Clone)]
pub enum ErrorSeverity {
    /// Error messages
    Error = 0,
    /// Information messages
    Info = 1,
    /// Warning messages
    Warning = 2,
    /// Used only in debug version of dll
    Debug = 4,
    /// All log types
    All = -1,
}

impl From<NvFlexErrorSeverity> for ErrorSeverity {
    #[inline]
    fn from(severity: NvFlexErrorSeverity) -> Self {
        match severity {
            nvflex_sys::eNvFlexLogError => Self::Error,
            nvflex_sys::eNvFlexLogInfo => Self::Info,
            nvflex_sys::eNvFlexLogWarning => Self::Warning,
            nvflex_sys::eNvFlexLogDebug => Self::Debug,
            _ => Self::All,
        }
    }
}

impl Into<NvFlexErrorSeverity> for ErrorSeverity {
    #[inline]
    fn into(self) -> i32 {
        match self {
            Self::Error => nvflex_sys::eNvFlexLogError,
            Self::Info => nvflex_sys::eNvFlexLogInfo,
            Self::Warning => nvflex_sys::eNvFlexLogWarning,
            Self::Debug => nvflex_sys::eNvFlexLogDebug,
            Self::All => nvflex_sys::eNvFlexLogAll,
        }
    }
}

#[cfg(feature = "logging")]
unsafe extern "C" fn default_error_callback(
    severity: NvFlexErrorSeverity,
    msg: *const i8,
    file: *const i8,
    line: i32,
) {
    let error = format!(
        "FleX error at {}:{} (severity {}): {}",
        rstr!(file),
        line,
        severity,
        rstr!(msg)
    );
    match severity {
        nvflex_sys::eNvFlexLogError => log::error!("{}", error),
        nvflex_sys::eNvFlexLogInfo => log::info!("{}", error),
        nvflex_sys::eNvFlexLogWarning => log::warn!("{}", error),
        nvflex_sys::eNvFlexLogDebug => log::debug!("{}", error),
        _ => log::info!("{}", error),
    }
}

#[derive(Debug, Clone)]
pub struct FlexContext {
    pub lib: *mut NvFlexLibrary,
    pub solver: *mut NvFlexSolver,
}

impl FlexContext {
    /// Initializes the FleX library and creates a solver. Returns `None` if the library could not
    /// be initialized. If the `logging` feature is enabled, FleX error messages will be logged using the `log` crate.
    ///
    /// # Arguments
    ///
    /// * `init_desc` - The `InitDesc` struct defining the device ordinal, D3D device/context and the type of D3D compute being used.
    /// * `solver_desc` - Optional solver description. If false, default solver settings are used.
    pub fn new(init_desc: Option<InitDesc>, solver_desc: Option<SolverDesc>) -> Option<Self> {
        unsafe {
            #[cfg(feature = "logging")]
            let lib = NvFlexInit(
                nvflex_sys::NV_FLEX_VERSION as _,
                Some(default_error_callback),
                if init_desc.is_some() {
                    let mut init_desc: NvFlexInitDesc = init_desc.unwrap().into();
                    &mut init_desc
                } else {
                    std::ptr::null_mut()
                },
            );

            #[cfg(not(feature = "logging"))]
            let lib = NvFlexInit(
                nvflex_sys::NV_FLEX_VERSION as _,
                None,
                if init_desc.is_some() {
                    let mut init_desc: NvFlexInitDesc = init_desc.unwrap().into();
                    &mut init_desc
                } else {
                    std::ptr::null_mut()
                },
            );

            if lib.is_null() {
                return None;
            }

            // create default solver settings
            let mut flex_solver_desc = std::mem::zeroed::<NvFlexSolverDesc>();
            NvFlexSetSolverDescDefaults(&mut flex_solver_desc);
            if solver_desc.is_some() {
                flex_solver_desc = solver_desc.unwrap().into();
            }

            // create solver and set default params
            let solver = NvFlexCreateSolver(lib, &flex_solver_desc);
            let params: NvFlexParams = SolverParams::DEFAULT_PARAMS.fixed().into();
            NvFlexSetParams(solver, &params);

            Some(Self { lib, solver })
        }
    }

    /// Update solver parameters.
    ///
    /// # Arguments
    ///
    /// * `params` - New solver parameters.
    /// * `fix_params` - Attempt to tweak solver params to be more or less sane. This can change
    /// `solid_rest_distance`, `collision_distance`, `particle_friction` and `shape_collision_margin`
    /// depending on their values.
    #[inline]
    pub fn set_params(&self, params: SolverParams, fix_params: bool) {
        let params: NvFlexParams = if fix_params {
            params.fixed().into()
        } else {
            params.into()
        };
        unsafe { NvFlexSetParams(self.solver, &params) }
    }

    /// Integrate particle solver forward in time.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time to integrate the solver forward in time, in seconds.
    /// * `substeps` - The time `dt` will be divided into the number of sub-steps given by this parameter.
    /// * `enable_timers` - Whether to enable per-kernel timers for profiling. Note that profiling can substantially slow down overall performance so this param should only be true in non-release builds.
    #[inline]
    pub fn tick(&mut self, dt: f32, substeps: i32, enable_timers: bool) {
        unsafe { NvFlexUpdateSolver(self.solver, dt, substeps, enable_timers) }
    }
}

impl Drop for FlexContext {
    fn drop(&mut self) {
        if !self.solver.is_null() {
            unsafe { NvFlexDestroySolver(self.solver) }
        }
        if !self.lib.is_null() {
            unsafe { NvFlexShutdown(self.lib) }
        }
    }
}
