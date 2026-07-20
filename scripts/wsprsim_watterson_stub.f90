subroutine watterson(c,npts,nsig,fs,delay,fspread)
! No-op stand-in for WSJT-X's lib/ft8/watterson.f90.
!
! wsprsimf.f90 only calls this on its *.c2 (nwav=0) output path, which
! build_wsprsim.sh never exercises -- this crate only wants the *.wav
! branch (nwav=1). Fortran external-procedure calls are resolved at
! link time regardless of which runtime branch actually executes, so
! the symbol still has to exist. The real watterson.f90 depends on
! four2a/FFTW for its fading-channel FFT convolution; since the call
! site is dead code for our purposes, this stub avoids pulling that
! dependency into a standalone build.
  complex c(0:npts-1)
  integer npts,nsig
  real fs,delay,fspread
  return
end subroutine watterson
