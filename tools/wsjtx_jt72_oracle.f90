! Minimal differential oracle for the legacy 72-bit message codec shared by
! JT4, JT9, and JT65.
!
! Compile this file together with the pinned WSJT-X v3.0.2 sources:
!
!   gfortran -I lib -o wsjtx-jt72-oracle \
!     lib/packjt.f90 lib/fmtmsg.f90 lib/grid2deg.f90 lib/deg2grid.f90 \
!     tools/wsjtx_jt72_oracle.f90
!
! The program reads one message per line and emits the normalized decode,
! WSJT-X message type, and the twelve packed six-bit words. It deliberately
! invokes upstream packmsg/unpackmsg directly so generated vectors do not
! depend on OpenDigi or mfsk-core behavior.
program wsjtx_jt72_oracle
  use packjt
  implicit none

  character(len=22) :: message
  character(len=22) :: decoded
  integer :: words(12)
  integer :: itype
  integer :: status

  do
    read (*, '(A)', iostat=status) message
    if (status /= 0) exit
    call packmsg(message, words, itype)
    call unpackmsg(words, decoded)
    write (*, '(I0,A,A,A,12(I0,1X))') itype, achar(9), trim(decoded), achar(9), words
  end do
end program wsjtx_jt72_oracle
