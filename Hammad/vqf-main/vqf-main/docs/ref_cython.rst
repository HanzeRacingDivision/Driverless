.. SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
..
.. SPDX-License-Identifier: MIT

.. module:: vqf
   :noindex:
.. mat:module:: matlab
    :noindex:

vqf.VQF
#######

..
    Note that the :members: are specified manually in order to ensure that the members are ordered by source which is
    otherwise not possible with Cython modules.

.. autoclass:: VQF
    :members: updateGyr,
        updateAcc,
        updateMag,
        update,
        updateBatch,
        updateBatchFullState,
        getQuat3D,
        getQuat6D,
        getQuat9D,
        getDelta,
        getBiasEstimate,
        setBiasEstimate,
        getRestDetected,
        getMagDistDetected,
        getRelativeRestDeviations,
        getMagRefNorm,
        getMagRefDip,
        setMagRef,
        setTauAcc,
        setTauMag,
        setMotionBiasEstEnabled,
        setRestBiasEstEnabled,
        setMagDistRejectionEnabled,
        setRestDetectionThresholds,
        params,
        coeffs,
        state,
        resetState,
        quatMultiply,
        quatConj,
        quatSetToIdentity,
        quatApplyDelta,
        quatRotate,
        norm,
        normalize,
        clip,
        gainFromTau,
        filterCoeffs,
        filterInitialState,
        filterAdaptStateForCoeffChange,
        filterStep,
        filterVec,
        matrix3SetToScaledIdentity,
        matrix3Multiply,
        matrix3MultiplyTpsFirst,
        matrix3MultiplyTpsSecond,
        matrix3Inv

    **Update Methods**

    .. autosummary::
        updateGyr
        updateAcc
        updateMag
        update
        updateBatch
        updateBatchFullState

    **Methods to Get/Set State**

    .. autosummary::
        getQuat3D
        getQuat6D
        getQuat9D
        getDelta
        getBiasEstimate
        setBiasEstimate
        getRestDetected
        getMagDistDetected
        getRelativeRestDeviations
        getMagRefNorm
        getMagRefDip
        setMagRef

    **Methods to Change Parameters**

    .. autosummary::
        setTauAcc
        setTauMag
        setMotionBiasEstEnabled
        setRestBiasEstEnabled
        setMagDistRejectionEnabled
        setRestDetectionThresholds

    **Access to Full Params/Coeffs/State**

    .. autosummary::
        params
        coeffs
        state
        resetState

    **Static Utility Functions**

    .. autosummary::
        quatMultiply
        quatConj
        quatSetToIdentity
        quatApplyDelta
        quatRotate
        norm
        normalize
        clip
        gainFromTau
        filterCoeffs
        filterInitialState
        filterAdaptStateForCoeffChange
        filterStep
        filterVec
        matrix3SetToScaledIdentity
        matrix3Multiply
        matrix3MultiplyTpsFirst
        matrix3MultiplyTpsSecond
        matrix3Inv
