! Oracle driver for the JTTY TBCC decode ladder (WSJT-X 3.2.0-rc1).
!
! Not upstream code. It links upstream's jtty_tbcc_* modules and prints, for a
! set of generated cases, (a) the correlations the receiver would hand to the
! decoder and (b) what the upstream ladder and list decoder return for them.
! A Rust port can then be tested at exactly the decoder boundary -- the same
! 4 x 46 correlations in, the same top-4 words out -- independently of any
! signal processing (sync search, peak-up, correlation), where the two
! implementations differ in ways that have nothing to do with the trellis.
!
! Correlation model (per frame; Es/N0 = snr_db, noise variance 1 per complex
! sample): every symbol is the sum of two half-symbol correlations.
!   true tone :  (A/2) exp(j phi_t) + n_half
!   other tones:                      n_half
! with A = 10**(snr_db/20), n_half complex Gaussian of variance 1/2, and
!   phi_t = phi_0 + t * dphi        (dphi = 0: phase constant over the frame)
! and the second half-symbol rotated by dhalf relative to the first (a
! frequency error inside the symbol: it shrinks |zsym| but not zhalf, which is
! why the ladder has a half-symbol rung at all).
! zsym = half_1 + half_2 (what jtty_correlate_payload_symbols returns as
! `correlations`); zhalf = sqrt(|half_1|^2 + |half_2|^2) as a real (its
! `half_correlations`).
!
! Usage: jtty_ladder_oracle seed  cases_per_condition
! Output is text, one block per case; floats are printed with 9 significant
! digits (es15.8), which round-trips a float32 exactly.

program jtty_ladder_oracle
  use, intrinsic :: iso_fortran_env, only: int32, int64, real32, real64
  use jtty_tbcc_code_profiles, only: JTTY_TBCC_PROFILE_1167_1545_80F
  use tbcc, only: tbcc_encode, PAYLOAD_BITS, TOTAL_K
  use jtty_tbcc_decoder, only: jtty_tbcc_decode, jtty_tbcc_decode_result
  use jtty_tbcc_list_decoder, only: jtty_tbcc_decoder_plan, &
       jtty_tbcc_decoder_workspace, jtty_tbcc_init_decoder_plan, &
       jtty_tbcc_init_decoder_workspace, jtty_tbcc_list_wava_optimized, &
       JTTY_TBCC_INFORMATION_BITS, JTTY_TBCC_MAX_HYPOTHESES
  implicit none

  integer, parameter :: NCOND = 11
  ! Es/N0 in dB, per-symbol phase advance and intra-symbol half rotation in
  ! rad. Chosen so that every rung of the ladder, and a total failure, occur
  ! (checked against the output by scripts/gen_jtty_vectors.sh, not assumed).
  real, parameter :: SNR_DB(NCOND) = [ 0.0, 2.0, 4.0,   2.0, 4.0,   4.0, 6.0,   4.0, 6.0, 8.0, 10.0 ]
  real, parameter :: DPHI(NCOND)   = [ 0.0, 0.0, 0.0,   0.25, 0.25, 1.0, 1.0,   0.0, 0.0, 0.0, 0.0 ]
  real, parameter :: DHALF(NCOND)  = [ 0.0, 0.0, 0.0,   0.0, 0.0,   0.0, 0.0,   2.5, 2.5, 2.5, 2.5 ]
  integer, parameter :: COH(3) = [1, 2, 4]

  type(jtty_tbcc_decoder_plan) :: plans(3)
  type(jtty_tbcc_decoder_workspace) :: workspaces(3)
  type(jtty_tbcc_decode_result) :: res
  complex(real32) :: zsym(0:3, TOTAL_K), zhalf(0:3, TOTAL_K)
  complex(real32) :: h1, h2
  integer(int32) :: payload(PAYLOAD_BITS), tones(TOTAL_K), dec(PAYLOAD_BITS)
  integer(int64) :: rng
  integer :: seed, ncases, icond, icase, icase_total, i, t, k, rung
  real(real64) :: amp, phi0, phi, u1, u2, g1, g2
  logical :: ok
  character(len=32) :: arg

  if (command_argument_count() /= 2) then
    print *, 'Usage: jtty_ladder_oracle seed cases_per_condition'
    stop 1
  end if
  call get_command_argument(1, arg); read (arg, *) seed
  call get_command_argument(2, arg); read (arg, *) ncases
  rng = int(seed, int64) * 2654435761_int64 + 88172645463325252_int64
  rng = iand(rng, 4294967295_int64)
  if (rng == 0_int64) rng = 2463534242_int64

  do i = 1, 3
    call jtty_tbcc_init_decoder_plan(plans(i), JTTY_TBCC_PROFILE_1167_1545_80F, &
         int(COH(i), int32), 4_int32, 2_int32, JTTY_TBCC_MAX_HYPOTHESES)
    call jtty_tbcc_init_decoder_workspace(workspaces(i), plans(i))
  end do

  write (*, '(a)') '# JTTY TBCC ladder oracle, WSJT-X 3.2.0-rc1 (see scripts/jttysim/)'
  write (*, '(a,i0,a,i0)') '# seed=', seed, ' cases_per_condition=', ncases
  write (*, '(a)') '# CASE id snr_db dphi dhalf ; PAYLOAD 34 bits ; ZS/ZH per symbol ; DEC ; RUNG lists'

  icase_total = 0
  do icond = 1, NCOND
    do icase = 1, ncases
      icase_total = icase_total + 1
      ! random 34-bit payload: 32 grammar bits, bit 33 = 0 (reserved), bit 34 free
      do i = 1, PAYLOAD_BITS
        if (i == 33) then
          payload(i) = 0
        else
          payload(i) = merge(1, 0, uniform() >= 0.5_real64)
        end if
      end do
      if (all(payload(1:32) == 0)) payload(1) = 1
      call tbcc_encode(payload, tones, JTTY_TBCC_PROFILE_1167_1545_80F)

      amp = 10.0_real64**(real(SNR_DB(icond), real64) / 20.0_real64)
      phi0 = 6.283185307179586_real64 * uniform()
      do t = 1, TOTAL_K
        phi = phi0 + real(t, real64) * real(DPHI(icond), real64)
        do k = 0, 3
          call gauss(g1, g2); h1 = cmplx(real(g1*0.5_real64, real32), real(g2*0.5_real64, real32), real32)
          call gauss(g1, g2); h2 = cmplx(real(g1*0.5_real64, real32), real(g2*0.5_real64, real32), real32)
          ! g ~ N(0,1) per component; scaling by 0.5 gives component variance
          ! 1/4, i.e. complex variance 1/2 per half-symbol.
          if (tones(t) == k) then
            h1 = h1 + cmplx(real(0.5_real64*amp*cos(phi), real32), real(0.5_real64*amp*sin(phi), real32), real32)
            h2 = h2 + cmplx(real(0.5_real64*amp*cos(phi + real(DHALF(icond), real64)), real32), &
                            real(0.5_real64*amp*sin(phi + real(DHALF(icond), real64)), real32), real32)
          end if
          zsym(k, t) = h1 + h2
          zhalf(k, t) = cmplx(sqrt(real(h1)**2 + aimag(h1)**2 + real(h2)**2 + aimag(h2)**2), 0.0, real32)
        end do
      end do

      write (*, '(a,i0,3(1x,f5.2))') 'CASE ', icase_total, SNR_DB(icond), DPHI(icond), DHALF(icond)
      write (*, '(a,1x,34i1)') 'PAYLOAD', payload
      do t = 1, TOTAL_K
        write (*, '(a,1x,i2,8(1x,es15.8))') 'ZS', t, &
             (real(zsym(k, t)), aimag(zsym(k, t)), k = 0, 3)
      end do
      do t = 1, TOTAL_K
        write (*, '(a,1x,i2,4(1x,es15.8))') 'ZH', t, (real(zhalf(k, t)), k = 0, 3)
      end do

      call jtty_tbcc_decode(zsym, zhalf, dec, ok, res)
      write (*, '(a,1x,l1,1x,a,i0,1x,a,i0,1x,a,i0,1x,a,l1,1x,a,i0,1x,a,es24.16)') &
           'DEC', ok, 'rungs=', res%evaluated_rung_count, 'block=', res%coherent_block_length, &
           'rank=', res%accepted_hypothesis_rank, 'half=', res%used_half_symbol_observation, &
           'pool=', res%circular_candidate_count, 'metric=', res%accepted_metric
      write (*, '(a,1x,34i1)') 'DECPAYLOAD', dec

      do rung = 1, 3
        call dump_list(rung, zsym, 'L')
      end do
      call dump_list(1, zhalf, 'H')
    end do
  end do

contains

  real(real64) function uniform() result(u)
    ! xorshift32 held in an int64 (masked), (x + 0.5) / 2**32
    integer(int64) :: x
    x = rng
    x = ieor(x, iand(ishft(x, 13), 4294967295_int64))
    x = ieor(x, ishft(x, -17))
    x = ieor(x, iand(ishft(x, 5), 4294967295_int64))
    rng = x
    u = (real(x, real64) + 0.5_real64) / 4294967296.0_real64
  end function uniform

  subroutine gauss(a, b)
    real(real64), intent(out) :: a, b
    real(real64) :: r, th
    r = sqrt(-2.0_real64 * log(uniform()))
    th = 6.283185307179586_real64 * uniform()
    a = r * cos(th)
    b = r * sin(th)
  end subroutine gauss

  subroutine dump_list(iplan, corr, tag)
    integer, intent(in) :: iplan
    complex(real32), intent(in) :: corr(0:3, TOTAL_K)
    character(len=1), intent(in) :: tag
    integer(int32) :: bits(JTTY_TBCC_INFORMATION_BITS, JTTY_TBCC_MAX_HYPOTHESES)
    integer(int64) :: ident(JTTY_TBCC_MAX_HYPOTHESES)
    real(real64) :: clean(JTTY_TBCC_MAX_HYPOTHESES), wava(JTTY_TBCC_MAX_HYPOTHESES)
    integer(int32) :: start(JTTY_TBCC_MAX_HYPOTHESES), ncand, npool
    logical :: crc(JTTY_TBCC_MAX_HYPOTHESES)
    integer :: r

    call jtty_tbcc_list_wava_optimized(plans(iplan), workspaces(iplan), corr, bits, ident, &
         clean, wava, start, crc, ncand, npool, prune_reserved_zero=.true.)
    write (*, '(a,a,i0,1x,a,i0,1x,a,i0)') 'RUNG ', tag, COH(iplan), 'count=', ncand, 'pool=', npool
    do r = 1, ncand
      write (*, '(a,1x,i0,1x,46i1,1x,l1,1x,es24.16,1x,es24.16,1x,i0)') 'H', r, &
           bits(:, r), crc(r), clean(r), wava(r), start(r)
    end do
  end subroutine dump_list

end program jtty_ladder_oracle
