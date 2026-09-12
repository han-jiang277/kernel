// Copyright (c) 2026 vivo Mobile Communication Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Network interface module for the layered architecture.
//!
//! `NetIface` is the L3 abstraction representing a network interface.
//! It owns a reference to a `LinkLayer` device (L2) and provides
//! type-safe control via `NetIfaceControl`.
//!
//! # Dyn-compatibility note
//!
//! `smoltcp::phy::Device` uses GATs (`RxToken`, `TxToken`) and is NOT
//! dyn-compatible. `LinkLayer` intentionally does NOT include `Device`
//! as a supertrait. Concrete types implement both traits separately,
//! and each `LinkLayer` impl handles the smoltcp poll cycle via
//! `poll_smoltcp()`. `NetIface` stores `Arc<RwLock<dyn LinkLayer>>`
//! (which IS dyn-compatible) for both control operations and smoltcp
//! poll dispatch.

pub(crate) use crate::net::iface::control::{
    InterfaceFlags, NetIfaceControl, NetIfaceError, NetIfaceResult,
};
use alloc::{string::String, sync::Arc, vec::Vec};
use smoltcp::{
    iface::{Interface, SocketHandle, SocketSet},
    socket::AnySocket,
    time::Instant,
};
use spin::{Mutex, RwLock};

use crate::net::{
    link::{HwAddr, Medium},
    smoltcp::link::SmoltcpDevice,
    socket::socket_err::SocketError,
};

struct SmoltcpState {
    iface: Option<Interface>,
    sockets: Option<SocketSet<'static>>,
}

/// L3 network interface.
///
/// Bridges the link layer (L2) with the protocol layer (L4).
/// Holds a single `Arc<RwLock<dyn SmoltcpDevice>>` — since
/// `SmoltcpDevice: LinkLayer`, both L2 control and smoltcp lifecycle
/// are available through one trait object.
pub struct NetIface {
    name: String,
    /// Link-layer device (L2 control + smoltcp lifecycle).
    link: Arc<RwLock<dyn SmoltcpDevice>>,
    /// smoltcp interface and socket set.
    smoltcp: Mutex<SmoltcpState>,
    /// Index into `LINK_REGISTRY`.
    link_index: usize,
}

impl NetIface {
    pub(crate) fn new(
        name: String,
        link: Arc<RwLock<dyn SmoltcpDevice>>,
        link_index: usize,
    ) -> Self {
        let (iface, sockets) = link.write().create_smoltcp_iface();

        NetIface {
            name,
            link,
            smoltcp: Mutex::new(SmoltcpState {
                iface: Some(iface),
                sockets: Some(sockets),
            }),
            link_index,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn link(&self) -> &Arc<RwLock<dyn SmoltcpDevice>> {
        &self.link
    }

    pub fn link_index(&self) -> usize {
        self.link_index
    }

    /// Add a smoltcp socket to this interface's socket set.
    pub fn add_socket<T: AnySocket<'static>>(&self, socket: T) -> Option<SocketHandle> {
        self.smoltcp
            .lock()
            .sockets
            .as_mut()
            .map(|sockets| sockets.add(socket))
    }

    /// Execute a closure with a smoltcp socket and Interface reference.
    ///
    /// Similar to the `with()` pattern used in TCP/UDP/ICMP sockets, but
    /// as a method on `NetIface` so sockets don't need to manage the locking.
    pub fn with_socket<T, F, R>(&self, handle: SocketHandle, f: F) -> Result<R, SocketError>
    where
        T: AnySocket<'static>,
        F: FnOnce(&mut T, &mut Interface) -> Result<R, SocketError>,
    {
        let mut smoltcp = self.smoltcp.lock();
        let SmoltcpState { iface, sockets } = &mut *smoltcp;
        let sockets = sockets.as_mut().ok_or(SocketError::InterfaceNoAvailable)?;
        let socket = sockets.get_mut::<T>(handle);
        let iface = iface.as_mut().ok_or(SocketError::InterfaceNoAvailable)?;
        f(socket, iface)
    }

    /// Remove a socket from this interface's socket set.
    pub fn remove_socket(&self, handle: SocketHandle) {
        if let Some(sockets) = self.smoltcp.lock().sockets.as_mut() {
            sockets.remove(handle);
        }
    }

    /// Check if the interface contains an IP address.
    pub fn contains_addr(&self, addr: smoltcp::wire::IpAddress) -> bool {
        if let Some(iface) = self.smoltcp.lock().iface.as_ref() {
            iface
                .ip_addrs()
                .iter()
                .any(|cidr| cidr.contains_addr(&addr))
        } else {
            false
        }
    }

    /// Poll the smoltcp interface for packet I/O.
    ///
    /// Dispatches to `SmoltcpDevice::poll_smoltcp()` which uses the concrete
    /// device type internally, keeping the L2 `LinkLayer` trait smoltcp-free.
    ///
    /// The `link.write()` lock is held only for the duration of a single
    /// ingress/egress cycle. Between ingress iterations the lock is dropped so
    /// that the Wi-Fi libnet80211 task can run and deliver beacon frames,
    /// preventing StationBeaconTimeout under CPU contention.
    pub fn poll(&self, timestamp: Instant) {
        // Drain up to INGRESS_BUDGET frames. Both `smoltcp` (spin::Mutex) and
        // `link` (spin::RwLock) are acquired and released on every iteration so
        // that other threads — UI, libnet80211 beacon handler — can make progress
        // between frames instead of spinning for the entire poll cycle.
        const INGRESS_BUDGET: usize = 8;
        for _ in 0..INGRESS_BUDGET {
            let done = {
                let mut state = self.smoltcp.lock();
                let SmoltcpState { iface, sockets } = &mut *state;
                if let (Some(iface), Some(sockets)) = (iface.as_mut(), sockets.as_mut()) {
                    let mut link = self.link.write();
                    link.poll_smoltcp_ingress_single(timestamp, iface, sockets)
                } else {
                    true
                }
            }; // smoltcp.lock() and link.write() both released here
            if done {
                break;
            }
        }
        // Egress pass: acquire both locks once for the full egress flush.
        let mut state = self.smoltcp.lock();
        let SmoltcpState { iface, sockets } = &mut *state;
        if let (Some(iface), Some(sockets)) = (iface.as_mut(), sockets.as_mut()) {
            self.link.write().poll_smoltcp_egress(timestamp, iface, sockets);
        }
    }

    /// Check whether the link has a packet queued for ingress processing.
    pub fn has_pending_rx(&self) -> bool {
        self.link.read().has_pending_rx()
    }

    /// Poll delay from smoltcp.
    ///
    /// `poll_delay` does not need the device (only `iface.poll_delay` is
    /// called), so it is handled directly without going through LinkLayer.
    pub fn poll_delay(&self, timestamp: Instant) -> Option<smoltcp::time::Duration> {
        // smoltcp reports no deadline for pure ingress polling. Wake the
        // network task immediately when the link has queued a frame.
        if self.has_pending_rx() {
            return Some(smoltcp::time::Duration::from_millis(0));
        }

        let mut smoltcp = self.smoltcp.lock();
        let SmoltcpState { iface, sockets } = &mut *smoltcp;
        if let (Some(iface), Some(sockets)) = (iface.as_mut(), sockets.as_ref()) {
            iface.poll_delay(timestamp, sockets)
        } else {
            None
        }
    }
}

impl core::fmt::Display for NetIface {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "NetIface({})", self.name)
    }
}
