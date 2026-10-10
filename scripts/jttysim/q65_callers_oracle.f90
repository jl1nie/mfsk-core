! Oracle driver for Q65's contest caller list (WSJT-X v3.3.0-beta1 q65_callers + q65_set_list2).
!
! Not upstream code. It links upstream's q65_callers module and q65_set_list2 and replays a
! script, so that a Rust port can be compared at exactly that boundary.
!
! Input, one command per line (fields separated by TAB):
!   R <TAB> freq_hz <TAB> now <TAB> message      q65_record_caller(callers,count,freq,msg,now)
!   X <TAB> now                                  q65_expire_callers(callers,count,now)
!   D <TAB> name                                 remove-by-name: q65_remove_caller
!   L <TAB> mycall <TAB> hiscall <TAB> hisgrid   q65_set_list2, then print the codewords
! Build (after `scripts/build_jt9_upstream.sh ../WSJT-X v3.3.0-beta1`, which leaves the
! libraries and modules in target/upstream/build-v3.3.0-beta1/):
!   B=target/upstream/build-v3.3.0-beta1
!   gfortran -O1 -fopenmp -I$B/fortran_modules_omp -J /tmp/q65co scripts/jttysim/q65_callers_oracle.f90 \
!     $B/libwsjt_fort_omp.a $B/libwsjt_cxx.a -lfftw3f_omp -lfftw3f -lfftw3_omp -lfftw3 -lstdc++ -lm -o q65_callers_oracle
!   grep -v '^#' embedded-poc/assets/golden/q65/callers_script.tsv | ./q65_callers_oracle > callers_beta1.txt
! Output: after R / X / D, `S <count>` and one `C call grid nsec nfreq` per caller;
!         after L, `W <ncw>` and one `K <index> <first 13 symbols>` per codeword.
program q65_callers_oracle
  use types, only: q3list
  use q65_callers
  implicit none
  type(q3list) :: callers(Q65_MAX_CALLERS)
  integer :: codewords(63, Q65_MAX_CODEWORDS)
  integer :: count, freq, now, ios, i, ncw, tab1, tab2, tab3
  character(len=1024) :: line
  character(len=12) :: mycall, hiscall
  character(len=6)  :: hisgrid
  character(len=37) :: msg
  character(len=16) :: tmp

  callers = q3list('', '', 0, 0, 0)
  count = 0
  do
     read(*, '(a)', iostat=ios) line
     if (ios /= 0) exit
     if (len_trim(line) < 1) cycle
     select case (line(1:1))
     case ('R')
        tab1 = index(line(3:), achar(9)) + 2
        tab2 = tab1 + index(line(tab1+1:), achar(9))
        read(line(3:tab1-1), *) freq
        read(line(tab1+1:tab2-1), *) now
        msg = line(tab2+1:)
        call q65_record_caller(callers, count, freq, msg, now)
        call dump_state()
     case ('X')
        read(line(3:), *) now
        call q65_expire_callers(callers, count, now)
        call dump_state()
     case ('D')
        call q65_remove_caller(callers, count, trim(line(3:)))
        call dump_state()
     case ('L')
        tab1 = index(line(3:), achar(9)) + 2
        tab2 = tab1 + index(line(tab1+1:), achar(9))
        mycall = line(3:tab1-1)
        hiscall = line(tab1+1:tab2-1)
        hisgrid = line(tab2+1:)
        call q65_set_list2(mycall, hiscall, hisgrid, callers, count, codewords, ncw)
        write(*, '(a,i0)') 'W ', ncw
        do i = 1, ncw
           write(*, '(a,i0,13(1x,i0))') 'K ', i, codewords(1:13, i)
        end do
     end select
  end do

contains
  subroutine dump_state()
    integer :: j
    write(*, '(a,i0)') 'S ', count
    do j = 1, count
       write(*, '(a,a,1x,a,1x,i0,1x,i0)') 'C ', trim(callers(j)%call), callers(j)%grid, callers(j)%nsec, callers(j)%nfreq
    end do
  end subroutine
end program q65_callers_oracle
