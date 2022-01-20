#ifndef DETECTCORNERS_HPP
#define DETECTCORNERS_HPP
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/boards_from_corners.h"
#include "config.hpp"


namespace cbdetectModified{

static bool filterBoardCorners(vector<vector<int>> &board, vector<cv::Point2d>& corners, cv::Size &size)
{
	vector<int> invalid_x_index, invalid_y_index;
	int m = board.size(), n = board[0].size();
	int i = 0;
	while (i<m)
	{
		int cnt = 0;
		for(int j = 0;j<n;++j){
			if (board[i][j] < 0)
				cnt++;
		}
		if (cnt > 3 || cnt > n / 3)
		{
			invalid_x_index.push_back(i);
			i++;
		}
		else break;
	}
	i = m -1;
	while (i>=0)
	{
		int cnt = 0;
		for(int j = 0;j<n;++j){
			if (board[i][j] < 0)
				cnt++;
		}
		if (cnt > 3 || cnt > n / 3)
		{
			invalid_x_index.push_back(i);
			i--;
		}
		else break;
	}
	i=0;
	while (i<n)
	{
		int cnt = 0;
		for(int j = 0;j<m;++j){
			if (board[j][i] < 0)
				cnt++;
		}
		if (cnt > 3 || cnt > m / 3)
		{
			invalid_y_index.push_back(i);
			i++;
		}
		else break;
	}
	i = n -1;
	while (i>=0)
	{
		int cnt = 0;
		for(int j = 0;j<m;++j){
			if (board[j][i] < 0)
				cnt++;
		}
		if (cnt > 3 || cnt > m / 3)
		{
			invalid_y_index.push_back(i);
			i--;
		}
		else break;
	}
	int new_m = m - invalid_x_index.size();
	int new_n = n  - invalid_y_index.size();
    //debug
    // if(1)
    // {
    //     cout<< "board size: "<< new_m << " "<<new_n<<endl;
    // }
    

    if(new_m!=size.height&& new_n!=size.width){
        return false;
    }

	vector<cv::Point2d> board_corners;
	for(int i=0;i<m;++i){
		if(find(invalid_x_index.begin(), invalid_x_index.end(), i)!= invalid_x_index.end()) continue;
		for(int j=0;j<n;++j){
			if(find(invalid_y_index.begin(), invalid_y_index.end(), j)!= invalid_y_index.end()) continue;
			if(board[i][j]<0){
				board_corners.push_back(cv::Point2d(-1.0,-1.0));// 缺失点，标记为(-1，-1)
			}else{
				board_corners.push_back(corners[board[i][j]]);
			}
		}
	}
	reverse(board_corners.begin(), board_corners.end());
	corners = board_corners;
	return true;
}

bool detectCorner(cv::Mat &input_image, vector<cv::Point2f> &image_corners)
{
    image_corners.clear();
    cbdetect::Corner corners;
    cbdetect::Params params;
    std::vector<cbdetect::Board> boards;
    cbdetect::find_corners(input_image, corners, params);
    cbdetect::boards_from_corners(input_image, corners, boards, params);
    if (boards.size() != 1)
		return false;
    vector<vector<int>> chessboard = boards[0].idx;
    vector<cv::Point2d> board_corners = corners.p;
    // 滤除边缘角点
    cv::Size board_size =  Config::checkerboardGridSize();
    if(!filterBoardCorners(chessboard, board_corners, board_size)){
        return false;
    }
    for(auto& corner: board_corners){
        image_corners.push_back(cv::Point2f(float(corner.x), float(corner.y)));
    }
    // debug 
    // cv::drawChessboardCorners(input_image, m_board_size, image_corners, true);
    // cv::imshow("debug", input_image);
    // cv::waitKey(0);
    return true;
}


void estimatePoseAndCompleteCorners(vector<cv::Point2f> &chessboard_corners, cv::Mat &rvec, cv::Mat &tvec)
{
    cv::Size board_size = Config::checkerboardGridSize();
    double square_size = Config::checkerboardSquareSize();
    int &board_width = board_size.width;
    int &board_height = board_size.height;
    std::vector<cv::Point3f> board_points, board_points_temp;;
    for (int i = 0; i < board_height; i++)
    {
        for (int j = 0; j < board_width; j++)
        {
            board_points.push_back(
                cv::Point3f(i * square_size, j* square_size, 0.0));
            if(chessboard_corners[i*board_width+j]==cv::Point2f(-1.0,-1.0)) continue;
            board_points_temp.push_back(
                cv::Point3f(i * square_size, j * square_size, 0.0));
        }
    }
    vector<cv::Point2f> chessboard_corners_temp;
    for(int i=0;i<chessboard_corners.size();++i){
        if(chessboard_corners[i]==cv::Point2f(-1.0,-1.0)) continue;
        chessboard_corners_temp.push_back(chessboard_corners[i]);
    }
    // debug 
    // cout<< board_points_temp.size()<<endl;
    // cout<< chessboard_corners_temp.size()<<endl;
    cv::Mat camera_matrix = Config::leftCameraMatrix();
    cv::Mat dist_coeffs = Config::leftCameraDistCoeffs();
    cv::solvePnP(cv::Mat(board_points_temp), cv::Mat(chessboard_corners_temp), camera_matrix, dist_coeffs, rvec, tvec);
    // debug 
    // if(1){
    //     cout<< rvec<< " "<< tvec<<endl;
    // }
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(board_points, rvec, tvec, camera_matrix , dist_coeffs, imagePoints);
    for(int i=0;i<board_points.size();++i){
        if(chessboard_corners[i]==cv::Point2f(-1.0,-1.0)){
            chessboard_corners[i] = imagePoints[i];
        }
    }
}




}// namespace cbdetectModified

#endif