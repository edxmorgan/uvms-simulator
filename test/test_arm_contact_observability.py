"""Native generated EKF: retain information without inventing contact evidence.

Synthetic limits below are test fixtures, not another runtime limit source.
Runtime limits still come only from the existing Xacro hardware description.
"""
import os
import re
from pathlib import Path

import numpy as np
import pytest

ca = pytest.importorskip('casadi')
ROOT = Path(__file__).resolve().parents[1]
# Use the native nominal parameter source; no sibling Simlab checkout required.
# This reader is test-only, and checks the exact generated model input size.
header = (ROOT / 'hardware/include/ros2_control_blue_reach_5/nominal_arm_parameters.hpp').read_text()
fields = ('link_masses', 'link_first_moments', 'link_inertias', 'viscous_friction',
          'coulomb_friction', 'static_friction', 'stribeck_velocity', 'gravity_vector',
          'payload_com', 'payload_mass', 'base_pose', 'world_pose', 'tip_offset_pose')
PARAMETERS = []
for field in fields:
    value = re.search(r'msg\.' + field + r'\s*=\s*([^;]+);', header).group(1)
    PARAMETERS.extend(float(v.strip()) for v in value.strip('{}').split(',') if v.strip())
assert len(PARAMETERS) == 81
LIB = os.environ.get('ARM_EKF_TEST_LIBRARY', str(ROOT / 'casadi_lib/libmDisturbanceEKF_next.so'))


@pytest.mark.parametrize('bits', range(1, 16))
def test_partial_contact_information_and_nullspace(bits):
    predict = ca.external('arm_disturbance_ekf_predict', LIB)
    update = ca.external('arm_disturbance_ekf_update_limited', LIB)
    x = np.r_[[3., 1., 1., 1.], np.zeros(8)]
    load = np.array([.03, -.03, .02, -.08])
    truth = x.copy(); truth[8:] = load
    y = np.array(predict(truth, np.zeros(4), PARAMETERS, .01)[0]).ravel()[:8]
    contact = np.array([(bits >> j) & 1 for j in range(4)])
    contact8 = np.tile(contact, 2)
    y[contact8 == 1] = x[:8][contact8 == 1]
    prior = np.diag([0.] * 8 + [1.] * 4)
    args = [x, prior, np.zeros(4), PARAMETERS, .01, y, np.zeros((12,12)),
            np.eye(8)*1e-7, np.zeros(4), 200., np.full(4,-100.), np.full(4,100.), contact]
    result = update(*args)
    posterior = np.array(result[1]); mean = np.array(result[0]).ravel()[8:]
    assert np.isfinite(posterior).all()
    assert np.linalg.eigvalsh(posterior).min() > -1e-10
    assert np.max(np.abs(posterior-posterior.T)) < 1e-12
    # The free-joint Jacobian determines which combinations can be learned.
    J = np.array(predict(x,np.zeros(4),PARAMETERS,.01)[1])[:8,8:]
    J = J[contact8 == 0]
    _, singular, V = np.linalg.svd(J, full_matrices=True)
    rank = int(np.sum(singular > 1e-8))
    null = V[rank:].T
    np.testing.assert_allclose(null.T @ posterior[8:,8:] @ null,
                               np.eye(4-rank), atol=1e-8)
    if bits != 15:
        assert np.trace(posterior[8:,8:]) < 3.5
        assert np.linalg.norm(mean) > .001
    else:
        np.testing.assert_allclose(mean, 0., atol=1e-12)
    # Clamped coordinates themselves must not masquerade as load evidence.
    changed_y = y.copy(); changed_y[contact8 == 1] += .05
    args[5] = changed_y
    changed = update(*args)
    np.testing.assert_allclose(np.array(changed[0]).ravel()[8:], mean, atol=1e-12)
    # Conservative controller admission remains closed during contact.
    np.testing.assert_array_equal(np.array(result[5]).ravel(), np.zeros(4))


def test_all_contacts_accumulate_process_uncertainty():
    update = ca.external('arm_disturbance_ekf_update_limited', LIB)
    x = np.r_[[3.,1.,1.,1.], np.zeros(8)]
    P = np.eye(12)
    Q = np.diag([1e-10]*4+[1e-8]*4+[.001]*4)
    for _ in range(50):
        result = update(x,P,np.zeros(4),PARAMETERS,.01,x[:8],Q,np.eye(8)*1e-6,
                        np.zeros(4),200.,[-100.]*4,[100.]*4,np.ones(4))
        x,P = np.array(result[0]).ravel(),np.array(result[1])
    np.testing.assert_allclose(x[8:],0.,atol=1e-12)
    np.testing.assert_allclose(P[8:,8:],np.eye(4)*1.05,atol=1e-10)


@pytest.mark.parametrize('seed', [None, 19, 71, 203])
def test_production_plant_sustained_contact_then_release(seed):
    plant = ca.external('Mnext_reg', str(ROOT / 'casadi_lib/libMnext.so'))
    update = ca.external('arm_disturbance_ekf_update_limited', LIB)
    truth = np.r_[[np.pi,.4727,.179,.01,0.],np.zeros(5)]
    x = np.r_[truth[:4],truth[5:9],np.zeros(4)]
    P = np.diag([.001]*8+[1.]*4)
    Q = np.diag([1e-8]*4+[1e-6]*4+[.001]*4)
    R = np.diag([1e-6]*4+[1e-5]*4)
    lo,hi = np.array([-100.,-100.,-100.,.01]),np.full(4,100.)
    rng = np.random.default_rng(seed)
    errors=[];contacts=[];variances=[]
    for k in range(800):
        dt = .01 + .002*np.sin(k*.1)
        load = np.array([.03,-.03,.02,-.08 if k<350 else .10])
        # A simple fixture controller, independent of the EKF estimate.
        u = np.array([10.,10.,10.,3.])*(np.array([np.pi,.4087,.18,0.])-truth[:4])-.5*truth[5:9]
        u = np.clip(u,[-1.5,-1.,-1.,-.54],[1.5,1.,1.,.54])
        nxt = np.array(plant(truth,np.r_[u+load,0.],dt,PARAMETERS,
                             300.,400.,0.,np.zeros(4),200.)).ravel()
        contact = ((nxt[:4]<=lo+1e-9)|(nxt[:4]>=hi-1e-9)).astype(float)
        for j in range(4):
            if nxt[j]<lo[j]:nxt[j],nxt[5+j]=lo[j],max(0.,nxt[5+j])
            if nxt[j]>hi[j]:nxt[j],nxt[5+j]=hi[j],min(0.,nxt[5+j])
        truth=nxt
        y=np.r_[truth[:4],truth[5:9]]
        if seed is not None:y=y+rng.normal(size=8)*np.sqrt(np.diag(R))
        result=update(x,P,u,PARAMETERS,dt,y,Q*dt,R,np.zeros(4),200.,lo,hi,contact)
        x,P=np.array(result[0]).ravel(),np.array(result[1])
        assert np.isfinite(x).all() and np.isfinite(P).all()
        assert np.linalg.eigvalsh(P).min()>-1e-9
        errors.append(x[8:]-load);contacts.append(contact);variances.append(P[8:,8:].copy())
    assert all(c[3]==1 for c in contacts[250:350])
    # Partial information is retained. With changing configuration and dt the
    # local nullspace can rotate, so compare weak vs strong directions rather
    # than asserting that one fixed prior variance must be preserved forever.
    eig=np.linalg.eigvalsh(variances[349])
    assert eig[-1]>10*eig[0]
    assert eig[0]<.01
    assert all(not any(c) for c in contacts[-100:])
    assert np.max(np.sqrt(np.mean(np.square(errors[-100:]),axis=0)))<.01
    assert np.max(np.diag(P)[8:])<.01
    assert eig[-1]>10*np.max(np.diag(P)[8:])
    print('contact eigenvalues',eig,'free RMSE',
          np.sqrt(np.mean(np.square(errors[-100:]),axis=0)))
