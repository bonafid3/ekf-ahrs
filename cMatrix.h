#ifndef CMATRIX_H
#define CMATRIX_H

#include "utils.h"

template <int NN>
class cFloat
{
	public:
	cFloat(){}
	float data[NN];
};

template <int ROW, int COL>
class cMatrix
{
	public:

	cMatrix<ROW,COL>() {
		if(ROW == COL && ROW > 1) {
			setToIdentity();
			} else {
			setToZero();
		}
	}

	void setToIdentity() {
		for(int r=0; r<ROW; r++) {
			for(int c=0; c<COL; c++) {
				if(r==c) { data[r][c] = 1; }
				else { data[r][c] = 0; }
			}
		}
	}

	void setToZero() {
		for(int r=0; r<ROW; r++) {
			for(int c=0; c<COL; c++) {
				data[r][c] = 0;
			}
		}
	}

	template<int OTHER_ROW, int OTHER_COL>
	void operator=(const cMatrix<OTHER_ROW,OTHER_COL>& other) {
		for(int r=0; r<ROW; r++) {
			for(int c=0; c<COL; c++) {
				if(OTHER_ROW-1 >= r && OTHER_COL-1 >= c)
				data[r][c] = other.data[r][c];
			}
		}
	}

	float  determinant() {
		const int N = ROW;

		cFloat<2*N*N> f;

		int off=0;

		for(int i=0; i<N*N; i++) {
			if(i > 0 && (i%N) == 0) off++;
			f.data[i] = data[i % N][(i+off)%N];
		}

		off=0;
		for(int i=0; i<N*N; i++) {
			if(i > 0 && (i%N) == 0) off++;
			f.data[i+N*N] = data[i % N][(N-1-(i%N)+off)%N];
		}

		float prod1, prod2, sumprod1 = 0, sumprod2 = 0;

		for(int i=0; i<N; i++) {
			prod1 = prod2 = 1;
			for(int j=0; j<N; j++) {
				prod1 *= f.data[i*N+j];
				prod2 *= f.data[i*N+j + N*N];
			}
			sumprod1 += prod1;
			sumprod2 += prod2;
		}

		return sumprod1 - sumprod2;
	}


	cMatrix<ROW,COL> inv() {
		cMatrix<ROW,COL> result;
		float invdet = 1 / determinant();
		return result;
	}

	template<int OTHER_ROW, int OTHER_COL>
	cMatrix<ROW,OTHER_COL> operator*(const cMatrix<OTHER_ROW,OTHER_COL> &other) {
		cMatrix<ROW,OTHER_COL> result;

		for( int i = 0; i < ROW; i++ ) {
			for( int j = 0; j < OTHER_COL; j++ ) {
				float dot_product = 0;
				for(int k = 0; k < COL; k++ ) {
					dot_product += data[i][k]*other.data[k][j];
				}
				result.data[i][j] = dot_product;
			}
		}
		return result;
	}

	cMatrix<ROW,COL> operator*(const float &scalar) {
		cMatrix<ROW,COL> result;
		for(int r=0; r<ROW; r++) {
			for(int c=0; c<COL; c++) {
				result.data[r][c] = data[r][c];
				result.data[r][c] *= scalar;
			}
		}
		return result;
	}

	cMatrix<ROW,COL> operator*(const int &scalar) {
		cMatrix<ROW,COL> result;
		for(int r=0; r<ROW; r++) {
			for(int c=0; c<COL; c++) {
				result.data[r][c] = data[r][c];
				result.data[r][c] *= scalar;
			}
		}
		return result;
	}

	cMatrix<ROW,COL> operator+(const cMatrix<ROW,COL> &other) {
		cMatrix<ROW,COL> result;
		for(int r=0; r<ROW; r++) {
			for(int c=0; c<COL; c++) {
				result.data[r][c] = data[r][c] + other.data[r][c];
			}
		}
		return result;
	}

	cMatrix<ROW,COL> operator-(const cMatrix<ROW,COL> &other) {
		cMatrix<ROW,COL> result;
		for(int r=0; r<ROW; r++) {
			for(int c=0; c<COL; c++) {
				result.data[r][c] = data[r][c] - other.data[r][c];
			}
		}
		return result;
	}

	cMatrix<COL,ROW> transposed() {
		cMatrix<COL,ROW> result;
		for(int r=0; r<ROW; r++) {
			for(int c=0; c<COL; c++) {
				result.data[c][r] = data[r][c];
			}
		}
		return result;
	}

	float data[ROW][COL];
};

class cMatrix3x3 : public cMatrix<3,3>
{
	public:
	cMatrix3x3(){}
};

class cVector3D : public cMatrix<3,1>
{
	public:
	cVector3D(){}
};

#endif // CMATRIX_H

